# stamping-dfm

**Python DFM (Design for Manufacturability) analysis library for progressive stamping dies.**

The first open-source Python toolkit for automated DFM checking of 
progressive die drawings (.dxf), targeted at electronics and consumer 
appliance manufacturers.

---

## What it does

- Parses DXF files using `ezdxf`
- Validates die geometry against industry standards (Tooling Handbook)
- Checks punch clearance, minimum hole diameter, hole spacing
- Detects wire-cut (EDM) layers and standard components
- Outputs structured JSON reports compatible with downstream systems

---

## Quickstart

```bash
pip install ezdxf
python geometry_validator.py --demo
```

---

## Validation Rules

| Rule | Description | Severity |
|------|-------------|----------|
| R01 | PRESSCAD layer coverage | WARNING |
| R02 | Upper/lower die completeness | WARNING |
| R03 | Wire-cut layer detection | INFO |
| R04 | Guide plate layer check | WARNING |
| R05 | Minimum punch diameter | CRITICAL |
| R06 | Punch-die clearance | CRITICAL |
| R07 | Minimum hole spacing | CRITICAL |
| R08 | Standard component detection | INFO |
| R09 | Tap pre-drill size detection | INFO |
| R10 | Material vs machining method | WARNING |

---

## Background

Built from real-world analysis of 300+ progressive die DXF files 
from a Guangdong-based metal stamping manufacturer.

Industry reference: DG Lianya Tooling Standards Handbook (东莞联亚,2010)

---

## Status

🚧 v0.1 — Work in progress

---

## Author

Derek Pan — Automation & Digitization Consultant
Shunde, Foshan, Guangdong, China
