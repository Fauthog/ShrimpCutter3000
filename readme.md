Assignments for Fuji drive in Alpha5 Software:
|CONT|Value|Function|Virtual or Hardware|
|-----|----|--------|-------------------|
|CONT1|11|RST|hardware|
|CONT2|6|Limit Switch 0 position|hardware|
|CONT3|7|+OT (over travel)|hardware|
|CONT4|10|EMG(emergency stop button)|hardware|
|CONT5|8|-OT (over travel)|hardware|
|CONT9|1|S-ON (servo on/energize)|virtual|
|CONT10|4|START|virtual|
|CONT11|5|ORG (homing/origin)|virtual|
|CONT12|23|Immediate Value Change|virtual|
|OUT6|2|in position|virtual|
|OUT7|82|command position completion|virtual|


All assignments are done in KnownGoodConfig_11092024.CSP

**send target relative position and speed:**

1. send bytearray to address 5101 with 4 registers, 8 bytes, 4 bytes position, 4 bytes speed
2. send START ON
3. send START OFF

**update speed:**

only works if started operation is not yet completed!
1. send bytearray to address 5101 with 4 registers, 8 bytes, 4 bytes position (ALL 0!), 4 bytes new speed
2. send Immediate Value Change ON (CONT12)
3. send Immediate Value Change OFF (CONT12)


