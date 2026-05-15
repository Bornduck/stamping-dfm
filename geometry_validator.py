"""
KL Stamping Die Geometry Validator — Phase 2 Rules Engine Prototype
====================================================================
Based on:
  - Lianyi TOOLING STANDARDS HANDBOOK (2010)
  - PRESSCAD layer naming convention
  - KL Phase 1 clustering analysis (223 DXF files)
  - Meeting notes from ZG (2026-04-18)

Author: DP (盘工)
Purpose: Demonstrate concrete Phase 2 rule engine capability to ZG
"""

import ezdxf
import math
import json
from dataclasses import dataclass, field, asdict
from typing import Optional
from collections import defaultdict


# ============================================================
# 1. DATA STRUCTURES
# ============================================================

@dataclass
class Circle:
    x: float
    y: float
    r: float
    layer: str
    diameter: float = field(init=False)

    def __post_init__(self):
        self.diameter = round(self.r * 2, 4)

    def distance_to(self, other: "Circle") -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def is_concentric_with(self, other: "Circle", tol=0.5) -> bool:
        return self.distance_to(other) < tol


@dataclass
class ValidationResult:
    rule_id: str
    rule_name: str
    severity: str          # CRITICAL / WARNING / INFO
    passed: bool
    message: str
    details: dict = field(default_factory=dict)


@dataclass
class DieReport:
    filename: str
    material: str
    thickness: float
    die_type: str          # continuous / progressive / single
    circles_extracted: int
    layers_found: list
    results: list[ValidationResult] = field(default_factory=list)
    summary: dict = field(default_factory=dict)

    def add(self, result: ValidationResult):
        self.results.append(result)

    def finalize(self):
        total   = len(self.results)
        passed  = sum(1 for r in self.results if r.passed)
        critical_fails = [r for r in self.results if not r.passed and r.severity == "CRITICAL"]
        warnings       = [r for r in self.results if not r.passed and r.severity == "WARNING"]
        self.summary = {
            "total_rules": total,
            "passed": passed,
            "failed": total - passed,
            "critical_failures": len(critical_fails),
            "warnings": len(warnings),
            "score": round(passed / total * 100, 1) if total else 0,
            "verdict": "PASS" if not critical_fails else "FAIL"
        }


# ============================================================
# 2. REFERENCE TABLES  (from Lianyi handbook)
# ============================================================

# Single-sided clearance as % of thickness, by material
CLEARANCE_TABLE = {
    "SPCC":          0.05,
    "SECC":          0.05,
    "SUS304_1/2H":   0.05,
    "SUS304_3/4H":   0.06,
    "SUS301_full":   0.06,
    "SUS301_extra":  0.07,
    "hard_cold_roll":0.08,
    "soft_cold_roll":0.04,
    "C5210_UH":      0.06,
    "brass_hard":    0.05,
    "soft_al":       0.04,
    "copper":        0.04,
}

# Minimum hole diameter as ratio of thickness (no guide / with guide)
MIN_HOLE_RATIO = {
    "hard_steel": (1.3, 0.5),
    "soft_steel": (1.0, 0.35),
    "brass":      (0.8, 0.30),
    "aluminum":   (0.6, 0.25),
}

# Tapping pre-drill diameters (mm)
TAP_PREDRILL = {
    "M2":   1.6,
    "M2.5": 2.1,
    "M3":   2.55,
    "M4":   3.35,
    "M5":   4.25,
    "M6":   5.10,
    "M8":   6.80,
    "M10":  8.50,
    "M12": 10.20,
}

# Standard component diameters to look for (from ZG's parts list)
STANDARD_COMPONENT_DIAMETERS = {
    "guide_post_outer":   [20, 22, 25, 28, 32, 38, 45],
    "inner_guide_post":   [8, 10, 12, 13, 16, 20, 25],
    "lift_pin":           [3, 4, 5, 6, 8, 10, 13],
    "spring_rectangular": [6, 8, 10, 12, 14, 16, 18, 20, 25, 30],
    "dowel_pin":          [6, 8, 10, 12],
    "stop_pin":           [12, 16, 20],
}

# PRESSCAD layer groups
PRESSCAD_UPPER = {"UP","UB","PH","DIE2","PS","COVER","U1","U2",
                  "UP_W","UB_W","PH_W","DIE2_W","PS_W",
                  "UP_O","UB_O","PH_O","DIE2_O","PS_O"}
PRESSCAD_LOWER = {"DIE","LB","PH2","LP","B1","B2","PS2",
                  "DIE_W","LB_W","LP_W",
                  "DIE_O","LB_O","LP_O","B2_O","B1_O"}
PRESSCAD_FUNCTIONAL = {"GUIDE","PUNCH","EJECT","EJECT2",
                       "MATER","PART","PRESS","SIDE"}
PRESSCAD_ALL = PRESSCAD_UPPER | PRESSCAD_LOWER | PRESSCAD_FUNCTIONAL


# ============================================================
# 3. DXF PARSER  (layer-aware circle extraction)
# ============================================================

def extract_circles_with_layers(dxf_path: str) -> tuple[list[Circle], list[str]]:
    """
    Extract ALL circles with their layer names.
    Also expands INSERT blocks to catch embedded circles.
    Returns (circles, layer_names)
    """
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    circles: list[Circle] = []
    layers_seen: set[str] = set()

    def process_entity(entity):
        layer = entity.dxf.layer.strip().upper()
        layers_seen.add(layer)
        if entity.dxftype() == "CIRCLE":
            circles.append(Circle(
                x=round(entity.dxf.center.x, 4),
                y=round(entity.dxf.center.y, 4),
                r=round(entity.dxf.radius, 4),
                layer=layer
            ))

    # Modelspace entities
    for e in msp:
        process_entity(e)
        # Expand INSERT blocks
        if e.dxftype() == "INSERT":
            try:
                block = doc.blocks[e.dxf.name]
                for be in block:
                    process_entity(be)
            except Exception:
                pass

    return circles, sorted(layers_seen)


# ============================================================
# 4. GEOMETRY ANALYSIS HELPERS
# ============================================================

def find_concentric_groups(circles: list[Circle], tol=0.5) -> list[list[Circle]]:
    """
    Group circles that share the same XY center (within tolerance).
    A group with circles on different layers = one physical hole location.
    """
    visited = [False] * len(circles)
    groups = []
    for i, c in enumerate(circles):
        if visited[i]:
            continue
        group = [c]
        visited[i] = True
        for j, other in enumerate(circles):
            if not visited[j] and c.is_concentric_with(other, tol):
                group.append(other)
                visited[j] = True
        if len(group) > 1:
            groups.append(group)
    return groups


def classify_hole(group: list[Circle],
                  material: str,
                  thickness: float) -> dict:
    """
    Given a group of concentric circles on different layers,
    infer the most likely standard component type.
    """
    layers = {c.layer for c in group}
    diameters = sorted(set(round(c.diameter, 2) for c in group))
    max_d = max(diameters)
    min_d = min(diameters)
    d_range = max_d - min_d

    result = {
        "layers": sorted(layers),
        "diameters": diameters,
        "candidate": "unknown",
        "confidence": "low",
        "notes": []
    }

    clearance_pct = CLEARANCE_TABLE.get(material, 0.05)
    expected_clearance = round(thickness * clearance_pct * 2, 3)  # bilateral

    # --- Guide post (large diameter, symmetric, corner position) ---
    if max_d >= 16 and any(d in STANDARD_COMPONENT_DIAMETERS["guide_post_outer"]
                           for d in [round(max_d)]):
        result["candidate"] = "outer_guide_post"
        result["confidence"] = "high"
        result["notes"].append(f"Ø{max_d} matches standard outer guide post series")

    # --- Inner guide post (small, DIE+PS layers) ---
    elif max_d <= 25 and layers & {"DIE","PS","PH","DIE2"}:
        if any(round(max_d) in STANDARD_COMPONENT_DIAMETERS["inner_guide_post"]
               for _ in [1]):
            result["candidate"] = "inner_guide_post"
            result["confidence"] = "medium"
            result["notes"].append(f"Ø{max_d} on die/stripper layers → inner guide")

    # --- Punch + die clearance (two concentric circles, PUNCH + DIE layers) ---
    elif "PUNCH" in layers and ("DIE" in layers or "DIE_W" in layers):
        measured_clearance = d_range
        expected = expected_clearance
        deviation_pct = abs(measured_clearance - expected) / expected * 100 if expected > 0 else 0
        result["candidate"] = "punch_clearance_pair"
        result["confidence"] = "high"
        result["notes"].append(
            f"Clearance: measured={measured_clearance:.3f}mm, "
            f"expected={expected:.3f}mm ({clearance_pct*100:.0f}%t), "
            f"deviation={deviation_pct:.1f}%"
        )

    # --- Tap pre-drill (specific diameters matching M-size bottom holes) ---
    else:
        for m_size, predrill in TAP_PREDRILL.items():
            if abs(min_d - predrill) < 0.15:
                result["candidate"] = f"tap_{m_size}"
                result["confidence"] = "medium"
                result["notes"].append(f"Ø{min_d} ≈ {m_size} pre-drill ({predrill}mm)")
                break

    # --- Lift pin ---
    if result["candidate"] == "unknown":
        if max_d <= 16 and layers & {"DIE","B2","LB"}:
            for d in STANDARD_COMPONENT_DIAMETERS["lift_pin"]:
                if abs(max_d - d) < 0.3:
                    result["candidate"] = "lift_pin"
                    result["confidence"] = "medium"
                    result["notes"].append(f"Ø{max_d} on lower die layers → lift pin")
                    break

    return result


# ============================================================
# 5. VALIDATION RULES
# ============================================================

def rule_presscad_layer_coverage(layers: list[str], die_type: str) -> ValidationResult:
    """
    R01: Check if file uses PRESSCAD standard layer naming.
    Continuous dies should have >10 standard layers.
    """
    std_layers = [l for l in layers if l in PRESSCAD_ALL]
    coverage = len(std_layers) / max(len(layers), 1)

    threshold = 0.30 if die_type == "continuous" else 0.15
    passed = coverage >= threshold

    return ValidationResult(
        rule_id="R01",
        rule_name="PRESSCAD Layer Coverage",
        severity="WARNING",
        passed=passed,
        message=(f"{'✓' if passed else '✗'} "
                 f"{len(std_layers)}/{len(layers)} layers are PRESSCAD standard "
                 f"({coverage*100:.0f}% coverage)"),
        details={"std_layers": std_layers, "coverage_pct": round(coverage*100,1)}
    )


def rule_upper_lower_balance(layers: list[str]) -> ValidationResult:
    """
    R02: A complete die drawing should have both upper AND lower die layers.
    Imbalance suggests partial drawing (e.g. only lower die sent to subcontractor).
    """
    has_upper = bool(set(layers) & PRESSCAD_UPPER)
    has_lower = bool(set(layers) & PRESSCAD_LOWER)
    passed = has_upper and has_lower

    return ValidationResult(
        rule_id="R02",
        rule_name="Upper/Lower Die Completeness",
        severity="WARNING",
        passed=passed,
        message=(f"{'✓' if passed else '✗'} "
                 f"Upper={'✓' if has_upper else '✗'}  "
                 f"Lower={'✓' if has_lower else '✗'}"),
        details={"has_upper": has_upper, "has_lower": has_lower}
    )


def rule_wire_cut_layers(layers: list[str]) -> ValidationResult:
    """
    R03: Wire-cut (_W) layers indicate EDM machining requirement.
    Flag for cost estimation: wire-cut jobs are significantly more expensive.
    """
    w_layers = [l for l in layers if l.endswith("_W")]
    has_wire_cut = len(w_layers) > 0

    return ValidationResult(
        rule_id="R03",
        rule_name="Wire-Cut (EDM) Detection",
        severity="INFO",
        passed=True,  # not a failure, just info
        message=(f"{'⚡ Wire-cut required' if has_wire_cut else '○ No wire-cut layers'}: "
                 f"{w_layers}"),
        details={"wire_cut_layers": w_layers, "count": len(w_layers)}
    )


def rule_guide_layer_present(layers: list[str]) -> ValidationResult:
    """
    R04: GUIDE layer should be present in continuous dies.
    KL Phase 1 finding: GUIDE coverage was 0% across all files — key data gap.
    """
    has_guide = "GUIDE" in layers or "GUIDE_W" in layers
    return ValidationResult(
        rule_id="R04",
        rule_name="Guide Plate Layer (GUIDE)",
        severity="WARNING",
        passed=has_guide,
        message=(f"{'✓ GUIDE layer found' if has_guide else '✗ No GUIDE layer — guide plate design missing or not drawn'}"),
        details={"has_guide": has_guide}
    )


def rule_min_punch_diameter(circles: list[Circle],
                             material: str,
                             thickness: float,
                             guided: bool = True) -> ValidationResult:
    """
    R05: Minimum punch hole diameter check.
    Source: Lianyi handbook Table 2-1, section 4.1
    """
    material_class = "soft_steel"
    if material in ("SUS304_1/2H","SUS304_3/4H","SUS301_full","SUS301_extra","hard_cold_roll"):
        material_class = "hard_steel"
    elif material in ("soft_al","aluminum"):
        material_class = "aluminum"
    elif material in ("brass_hard","copper"):
        material_class = "brass"

    ratio = MIN_HOLE_RATIO[material_class][1 if guided else 0]
    min_allowed = ratio * thickness

    punch_circles = [c for c in circles if c.layer == "PUNCH"]
    violations = [c for c in punch_circles if c.diameter < min_allowed]

    passed = len(violations) == 0
    return ValidationResult(
        rule_id="R05",
        rule_name="Minimum Punch Diameter",
        severity="CRITICAL",
        passed=passed,
        message=(f"{'✓' if passed else '✗'} "
                 f"Min allowed: Ø{min_allowed:.3f}mm "
                 f"({ratio}×t, {material_class}, {'guided' if guided else 'unguided'}). "
                 f"Violations: {len(violations)}/{len(punch_circles)} punch holes"),
        details={
            "min_allowed_mm": round(min_allowed, 3),
            "punch_holes_checked": len(punch_circles),
            "violations": [{"x":c.x,"y":c.y,"d":c.diameter} for c in violations[:5]]
        }
    )


def rule_punch_die_clearance(circles: list[Circle],
                              material: str,
                              thickness: float) -> ValidationResult:
    """
    R06: Validate punch-to-die clearance for concentric PUNCH/DIE circle pairs.
    Source: Lianyi handbook section 5.8 (clearance table)
    """
    expected_pct = CLEARANCE_TABLE.get(material, 0.05)
    expected_bilateral = expected_pct * 2 * thickness
    tolerance = expected_bilateral * 0.3   # ±30% tolerance band

    punch_circles = [c for c in circles if c.layer == "PUNCH"]
    die_circles   = [c for c in circles if c.layer in ("DIE","DIE_W")]

    pairs_checked = 0
    violations = []
    for pc in punch_circles:
        for dc in die_circles:
            if pc.is_concentric_with(dc, tol=0.5):
                measured = dc.diameter - pc.diameter
                pairs_checked += 1
                if abs(measured - expected_bilateral) > tolerance:
                    violations.append({
                        "x": pc.x, "y": pc.y,
                        "punch_d": pc.diameter,
                        "die_d": dc.diameter,
                        "measured_clearance": round(measured, 4),
                        "expected_clearance": round(expected_bilateral, 4),
                    })

    passed = len(violations) == 0
    return ValidationResult(
        rule_id="R06",
        rule_name="Punch-Die Clearance Validation",
        severity="CRITICAL",
        passed=passed,
        message=(f"{'✓' if passed else '✗'} "
                 f"Expected bilateral clearance: {expected_bilateral:.3f}mm "
                 f"({expected_pct*100:.0f}%t × 2) for {material}. "
                 f"Pairs checked: {pairs_checked}. Violations: {len(violations)}"),
        details={
            "expected_pct": expected_pct,
            "expected_bilateral_mm": round(expected_bilateral,4),
            "pairs_checked": pairs_checked,
            "violations": violations[:5]
        }
    )


def rule_hole_spacing(circles: list[Circle], thickness: float) -> ValidationResult:
    """
    R07: Minimum hole-to-hole spacing.
    Source: Lianyi handbook section 4.2 — spacing ≥ 2t, min 3mm
    """
    min_spacing = max(2 * thickness, 3.0)
    punch_circles = [c for c in circles if c.layer == "PUNCH"]
    violations = []

    for i, a in enumerate(punch_circles):
        for b in punch_circles[i+1:]:
            if a.is_concentric_with(b, tol=0.5):
                continue   # same hole, skip
            dist = a.distance_to(b) - a.r - b.r  # edge-to-edge
            if dist < min_spacing:
                violations.append({
                    "hole1": {"x":a.x,"y":a.y,"d":a.diameter},
                    "hole2": {"x":b.x,"y":b.y,"d":b.diameter},
                    "edge_gap": round(dist,3),
                    "required": round(min_spacing,3)
                })

    passed = len(violations) == 0
    return ValidationResult(
        rule_id="R07",
        rule_name="Minimum Hole Spacing",
        severity="CRITICAL",
        passed=passed,
        message=(f"{'✓' if passed else '✗'} "
                 f"Min edge-to-edge: {min_spacing:.1f}mm (2t={2*thickness:.1f}mm). "
                 f"Violations: {len(violations)}"),
        details={"min_spacing_mm": min_spacing, "violations": violations[:5]}
    )


def rule_standard_component_detection(circles: list[Circle],
                                       material: str,
                                       thickness: float) -> ValidationResult:
    """
    R08: Identify likely standard components from concentric circle groups.
    This is the Phase 2 'standard part recognition' ZG asked for.
    Currently: layer-aware spatial clustering → component inference.
    """
    groups = find_concentric_groups(circles, tol=0.5)
    findings = []
    for g in groups:
        if len(g) >= 2:
            classification = classify_hole(g, material, thickness)
            if classification["candidate"] != "unknown":
                findings.append(classification)

    candidates = {}
    for f in findings:
        c = f["candidate"]
        candidates[c] = candidates.get(c, 0) + 1

    passed = len(findings) > 0
    return ValidationResult(
        rule_id="R08",
        rule_name="Standard Component Detection",
        severity="INFO",
        passed=passed,
        message=(f"Found {len(findings)} likely standard component locations: "
                 f"{json.dumps(candidates, ensure_ascii=False)}"),
        details={"component_counts": candidates, "findings": findings[:10]}
    )


def rule_tap_predrill_check(circles: list[Circle]) -> ValidationResult:
    """
    R09: Check if circles matching tap pre-drill sizes are present.
    Helps confirm threaded hole intentions from Lianyi handbook table 5.9.
    """
    results = {}
    for c in circles:
        for m_size, predrill in TAP_PREDRILL.items():
            if abs(c.diameter - predrill) < 0.15:
                results[m_size] = results.get(m_size, 0) + 1

    return ValidationResult(
        rule_id="R09",
        rule_name="Tap Pre-Drill Size Detection",
        severity="INFO",
        passed=True,
        message=f"Detected tap sizes: {results if results else 'none found'}",
        details={"tap_sizes_detected": results}
    )


def rule_material_layer_consistency(layers: list[str],
                                     material: str) -> ValidationResult:
    """
    R10: SUS304 / stainless steel dies should use SKH51 punch material
    and tighter tolerances. Flag if material is stainless but no wire-cut layers.
    """
    is_stainless = "SUS" in material
    has_wire_cut = any(l.endswith("_W") for l in layers)

    if is_stainless and not has_wire_cut:
        passed = False
        msg = (f"✗ Stainless steel ({material}) detected but no wire-cut layers found. "
               f"Stainless dies typically require EDM wire-cut for punch/die precision.")
    else:
        passed = True
        msg = f"✓ Material ({material}) and machining method appear consistent."

    return ValidationResult(
        rule_id="R10",
        rule_name="Material vs. Machining Method",
        severity="WARNING",
        passed=passed,
        message=msg,
        details={"material": material, "is_stainless": is_stainless,
                 "has_wire_cut": has_wire_cut}
    )


# ============================================================
# 6. MAIN VALIDATOR
# ============================================================

def validate_die(dxf_path: str,
                 material: str = "SPCC",
                 thickness: float = 1.0,
                 die_type: str = "continuous",
                 guided: bool = True) -> DieReport:
    """
    Run full validation on a DXF file.

    Parameters
    ----------
    dxf_path  : path to .dxf file
    material  : material code (see CLEARANCE_TABLE keys)
    thickness : sheet thickness in mm
    die_type  : 'continuous' | 'progressive' | 'single'
    guided    : whether punch has stripper plate guidance
    """
    import os
    filename = os.path.basename(dxf_path)

    # Extract geometry
    circles, layers = extract_circles_with_layers(dxf_path)

    report = DieReport(
        filename=filename,
        material=material,
        thickness=thickness,
        die_type=die_type,
        circles_extracted=len(circles),
        layers_found=layers
    )

    # Run all rules
    report.add(rule_presscad_layer_coverage(layers, die_type))
    report.add(rule_upper_lower_balance(layers))
    report.add(rule_wire_cut_layers(layers))
    report.add(rule_guide_layer_present(layers))
    report.add(rule_min_punch_diameter(circles, material, thickness, guided))
    report.add(rule_punch_die_clearance(circles, material, thickness))
    report.add(rule_hole_spacing(circles, thickness))
    report.add(rule_standard_component_detection(circles, material, thickness))
    report.add(rule_tap_predrill_check(circles))
    report.add(rule_material_layer_consistency(layers, material))

    report.finalize()
    return report


# ============================================================
# 7. REPORTING
# ============================================================

def print_report(report: DieReport):
    """Pretty-print a validation report to console."""
    w = 65
    print("=" * w)
    print(f"  KL DIE GEOMETRY VALIDATOR — Phase 2 Prototype")
    print("=" * w)
    print(f"  File     : {report.filename}")
    print(f"  Material : {report.material}  |  t = {report.thickness}mm")
    print(f"  Die type : {report.die_type}")
    print(f"  Circles  : {report.circles_extracted} extracted")
    print(f"  Layers   : {len(report.layers_found)} total, "
          f"{sum(1 for l in report.layers_found if l in PRESSCAD_ALL)} PRESSCAD")
    print("-" * w)

    severity_order = {"CRITICAL": 0, "WARNING": 1, "INFO": 2}
    sorted_results = sorted(report.results,
                            key=lambda r: (severity_order.get(r.severity,9), r.rule_id))

    for r in sorted_results:
        icon = {"CRITICAL":"🔴","WARNING":"🟡","INFO":"🔵"}.get(r.severity,"⚪")
        status = "PASS" if r.passed else "FAIL"
        print(f"\n  {icon} [{r.rule_id}] {r.rule_name}  [{status}]")
        print(f"     {r.message}")

    print("\n" + "=" * w)
    s = report.summary
    verdict_icon = "✅" if s["verdict"] == "PASS" else "❌"
    print(f"  {verdict_icon} VERDICT: {s['verdict']}   "
          f"Score: {s['score']}%   "
          f"({s['passed']}/{s['total_rules']} rules passed)")
    print(f"  Critical failures: {s['critical_failures']}   "
          f"Warnings: {s['warnings']}")
    print("=" * w)


def export_json(report: DieReport, output_path: str):
    """Export full report as JSON for downstream (e.g. Java API, frontend)."""
    data = asdict(report)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"  → JSON report saved: {output_path}")


# ============================================================
# 8. DEMO / ENTRY POINT
# ============================================================

def demo_without_file():
    """
    Demo mode: simulate a validation result without a real DXF file.
    Shows what the output looks like for ZG's presentation.
    """
    print("\n" + "="*65)
    print("  DEMO MODE — Simulated KL3325 validation")
    print("  (Run validate_die('KL3325.dxf', ...) for real file)")
    print("="*65)

    report = DieReport(
        filename="KL3325-demo-simulation.dxf",
        material="SUS304_1/2H",
        thickness=0.8,
        die_type="continuous",
        circles_extracted=847,
        layers_found=["DIE","DIE_W","DIE_O","B2","B2_O","UP","UB",
                      "PH","PS","PS_W","PUNCH","GUIDE","MATER","PART",
                      "DIM","B1","LP","0","CENTER"]
    )

    # Simulate rule results based on KL Phase 1 findings
    report.add(ValidationResult("R01","PRESSCAD Layer Coverage","WARNING",True,
        "✓ 14/19 layers are PRESSCAD standard (73.7% coverage)",
        {"std_layers":["DIE","DIE_W","B2","UP","UB","PH","PS","PS_W",
                       "PUNCH","GUIDE","MATER","PART","B1","LP"],
         "coverage_pct":73.7}))

    report.add(ValidationResult("R02","Upper/Lower Die Completeness","WARNING",True,
        "✓ Upper=✓  Lower=✓",
        {"has_upper":True,"has_lower":True}))

    report.add(ValidationResult("R03","Wire-Cut (EDM) Detection","INFO",True,
        "⚡ Wire-cut required: ['DIE_W', 'PS_W']",
        {"wire_cut_layers":["DIE_W","PS_W"],"count":2}))

    report.add(ValidationResult("R04","Guide Plate Layer (GUIDE)","WARNING",True,
        "✓ GUIDE layer found",
        {"has_guide":True}))

    report.add(ValidationResult("R05","Minimum Punch Diameter","CRITICAL",True,
        "✓ Min allowed: Ø0.400mm (0.5×t, hard_steel, guided). "
        "Violations: 0/23 punch holes",
        {"min_allowed_mm":0.400,"punch_holes_checked":23,"violations":[]}))

    report.add(ValidationResult("R06","Punch-Die Clearance Validation","CRITICAL",False,
        "✗ Expected bilateral clearance: 0.080mm (5%t×2) for SUS304_1/2H. "
        "Pairs checked: 23. Violations: 3",
        {"expected_pct":0.05,"expected_bilateral_mm":0.08,"pairs_checked":23,
         "violations":[
             {"x":125.5,"y":88.3,"punch_d":3.0,"die_d":3.12,
              "measured_clearance":0.12,"expected_clearance":0.08},
             {"x":145.2,"y":88.3,"punch_d":3.0,"die_d":3.13,
              "measured_clearance":0.13,"expected_clearance":0.08},
             {"x":165.0,"y":88.3,"punch_d":2.5,"die_d":2.61,
              "measured_clearance":0.11,"expected_clearance":0.08},
         ]}))

    report.add(ValidationResult("R07","Minimum Hole Spacing","CRITICAL",True,
        "✓ Min edge-to-edge: 1.6mm (2t=1.6mm). Violations: 0",
        {"min_spacing_mm":1.6,"violations":[]}))

    report.add(ValidationResult("R08","Standard Component Detection","INFO",True,
        'Found 18 likely standard component locations: '
        '{"tap_M6": 8, "tap_M8": 4, "lift_pin": 4, "inner_guide_post": 2}',
        {"component_counts":{"tap_M6":8,"tap_M8":4,"lift_pin":4,"inner_guide_post":2},
         "findings":[
             {"layers":["DIE","DIE_W"],"diameters":[5.0,5.1],
              "candidate":"tap_M6","confidence":"medium",
              "notes":["Ø5.0 ≈ M6 pre-drill (5.10mm)"]},
             {"layers":["DIE","B2"],"diameters":[8.0,8.0],
              "candidate":"lift_pin","confidence":"medium",
              "notes":["Ø8.0 on lower die layers → lift pin"]},
         ]}))

    report.add(ValidationResult("R09","Tap Pre-Drill Size Detection","INFO",True,
        'Detected tap sizes: {"M6": 8, "M8": 4}',
        {"tap_sizes_detected":{"M6":8,"M8":4}}))

    report.add(ValidationResult("R10","Material vs. Machining Method","WARNING",True,
        "✓ Material (SUS304_1/2H) and machining method appear consistent.",
        {"material":"SUS304_1/2H","is_stainless":True,"has_wire_cut":True}))

    report.finalize()
    print_report(report)
    return report


if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 2:
        # Real file mode
        dxf_path = sys.argv[1]
        material  = sys.argv[2] if len(sys.argv) > 2 else "SPCC"
        thickness = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
        die_type  = sys.argv[4] if len(sys.argv) > 4 else "continuous"

        print(f"\nValidating: {dxf_path}")
        report = validate_die(dxf_path, material, thickness, die_type)
        print_report(report)
        export_json(report, dxf_path.replace(".dxf", "_validation.json"))
    else:
        # Demo mode — no file needed
        demo_without_file()
