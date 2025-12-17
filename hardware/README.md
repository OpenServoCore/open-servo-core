# KiCad Component Management

## Component Linkage System

### Directory Structure

```
hardware/
├── shared.kicad_sym        # Symbol library
├── shared.pretty/           # Footprint library
├── shared.3dshapes/         # 3D models
├── templates/jlc4l_1v6mm/   # Template project (canonical reference)
└── boards/*/                # Board projects
```

### Linkage Patterns

#### 1. Project → Libraries

Projects link to shared libraries via `sym-lib-table` and `fp-lib-table`:

- Symbol library: `${KIPRJMOD}/../../shared.kicad_sym`
- Footprint library: `${KIPRJMOD}/../../shared.pretty`

#### 2. Symbol → Footprint

Symbols in `shared.kicad_sym` reference footprints using library prefix:

- Format: `"shared:FOOTPRINT_NAME"`
- Example: `"shared:UQFN-20_L3.0-W3.0-P0.40-BL-EP1.7"`

#### 3. Footprint → 3D Model

Footprints in `shared.pretty/` reference 3D models using relative paths:

- Format: `${KIPRJMOD}/../../shared.3dshapes/MODEL_NAME.wrl`
- Example: `${KIPRJMOD}/../../shared.3dshapes/UQFN-20_L3.0-W3.0-P0.40-BL-EP1.7.wrl`

## Component Acquisition Strategy

1. Navigate to a board or template directory:

   ```bash
   cd hardware/templates/jlc4l_1v6mm
   ```

2. Download component:

   ```bash
   easyeda2kicad --full --lcsc_id <LCSC_ID> --output ../../shared --overwrite --project-relative
   ```

3. Fix 3D model paths in footprint files:
   - Find: `${KIPRJMOD}././shared.3dshapes`
   - Replace: `${KIPRJMOD}/../../shared.3dshapes`

## Post-Import Checklist

- [ ] Symbol has correct footprint reference (`"shared:FOOTPRINT_NAME"`)
- [ ] Footprint has correct 3D model path (`${KIPRJMOD}/../../shared.3dshapes/...`)
- [ ] Pin types are properly assigned (not "unspecified")
- [ ] 3D model displays correctly in KiCad 3D viewer
- [ ] Component source documented (LCSC part number)
