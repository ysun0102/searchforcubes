#/bin/bash


make clean ; make


# ./search_TRIVIUM.run 840 -1 0 | tee search_cr1_r840_k0.log

# ./search_TRIVIUM.run 840 -1 1 | tee search_cr1_r840_k1.log

# ./search_TRIVIUM.run 840 -1 2 | tee search_cr1_r840_k2.log


./search_TRIVIUM.run 840 -1 0 | tee search_cr2_840_k0.log

./search_TRIVIUM.run 840 -1 1 | tee search_cr2_840_k1.log

./search_TRIVIUM.run 840 -1 2 | tee search_cr2_r840_k2.log

