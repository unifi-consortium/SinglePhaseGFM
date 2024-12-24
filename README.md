# SinglePhaseProject

## Required Software & Packages

1. TI Code Composer Studio v10
2. TI C2000Ware v5.0.0.00

## Environment Configuration and Properties

### Project: SinglePhaseGFM_1500W_120V

#### Include Options
- Include search paths:
  - `${COM_TI_C2000WARE_SOFTWARE_PACKAGE_INCLUDE_PATH}`
  - `${CG_TOOL_ROOT}/include`
  - `${INSTALLROOT_F2837XD}/headers/include`
  - `${INSTALLROOT_F2837XD}/common/include`

- Preinclude files:
  - Not specified.

#### Library and Command Files
- Library files:
  - `rts2800_fpu32.lib`
- Command files:
  - `2837xD_RAM_lnk_cpu1.cmd`
  - `F2837xD_Headers_nonBIOS_cpu1.cmd`

- Library search paths:
  - `${COM_TI_C2000WARE_SOFTWARE_PACKAGE_LIBRARY_PATH}`
  - `${CG_TOOL_ROOT}/lib`
  - `${CG_TOOL_ROOT}/include`
  - `${INSTALLROOT_F2837XD}/common/cmd`
  - `${INSTALLROOT_F2837XD}/headers/cmd`

---

For more detailed information about the configuration and settings, refer to the project files.




# SinglePhaseGFM
Single phase GFM embedded firmware

# TODO:
Use strict HAL so code does not need to run on C2000
