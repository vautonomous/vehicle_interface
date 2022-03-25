leo_vcu_driver {#leo_vcu_driver-package-design}
===========

This is the design document for the `leo_vcu_driver` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
It is designed to provide communication between Autoware.Auto and Low Level Controller


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
It takes inputs from Autoware.Auto and LLC, and calculates the appropriate outputs for both.
It behaves like buffer for state messages but It makes transforms for steering angles and steering wheel angles.

## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
https://1drv.ms/x/s!Aom6BtFOoqUVhWQJ965270XJi4Wq?e=lW0yJg

## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->
