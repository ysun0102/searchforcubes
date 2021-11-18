## Contents

1. Code for verifying parts of the superpoly for 843-round Trivium: trivium_verify.cpp.

2. Monomials in the superpoly for the cube {0, 1, ..., 79}/{30, 76}: superpoly/raw_monomials, superpoly/superpoly_monomials.

3. Superpoly of 843 round trivium in pdf form: superpoly_843round.pdf

3. Logs for computed results: log/trivium_verify_843_1_key*.log.


## Usage of the Codes

### Compile 

Please edit "Makefile" according to your configuration. Then type 

`make`

to compile the codes.

### Run the program

After you compile the code, please type 

`./trivium_verify [ROUND] [INDEX] [KEY_INDEX]`  

for run.

The possible combinations of (ROUND, INDEX, KEY_INDEX) are listed as follows, 
1. ROUND = 843, INDEX = 1, KEY_INDEX = 0:
    Recover part of the superpoly for the cube {0,1,...,79}/{30, 76} of 843-round Trivium, which consists of all monomials involving k0.

2. ROUND = 843, INDEX = 1, KEY_INDEX = 2:
    Recover part of the superpoly for the cube {0,1,...,79}/{30, 76} of 843-round Trivium, which consists of all monomials involving k2. The output should be a single monomial {k2}, which means no other monomials in the super polynomial involve the variable k2, so this superpoly is a balance polynomial.
