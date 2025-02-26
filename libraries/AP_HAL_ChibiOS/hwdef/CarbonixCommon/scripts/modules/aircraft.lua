-- This version of this module is only for SITL. In the real aircraft, we
-- hard-code the type. In SITL, for convenience, we infer it from params.

-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('This copy of aircraft.lua was designed for SITL'))

efi_type = param:get('EFI_TYPE')
assert(efi_type)

if efi_type == 0 then
  return "Volanti"
else
    return "Ottano"
end
