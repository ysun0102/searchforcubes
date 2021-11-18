#/bin/bash


make clean ; make

./retrieve_TRIVIUM.run 840 -1 -1 10 26 | tee retrieve_r840_v10_v26.log

./retrieve_TRIVIUM.run 840 -1 -1 6 9 | tee retrieve_r840_v6_v9.log

./retrieve_TRIVIUM.run 840 -1 -1 10 32 | tee retrieve_r840_v10_v32.log

./retrieve_TRIVIUM.run 840 -1 -1 7 46 | tee retrieve_r840_v7_v46.log

./retrieve_TRIVIUM.run 840 -1 -1 10 33 | tee retrieve_r840_v10_v33.log

./retrieve_TRIVIUM.run 840 -1 -1 7 61 | tee retrieve_r840_v7_v61.log

