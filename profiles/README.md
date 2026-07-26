# Device Mapping Profiles

These JSON files can be uploaded from the Web mapping page with
`Import Device Profile`.

A profile describes logical Modbus points rather than individual 16-bit
words. The gateway uses `data_type`, `byte_order`, `scale`, `offset`, and
`unit` to construct the engineering value:

```text
engineering_value = decode(register_words) * scale + offset
```

Importing a profile replaces raw discovery mappings only for the slave IDs
contained in that profile and leaves unrelated devices unchanged.
