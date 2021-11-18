## Uasage:

### Revise "Makefile" if necessary

1. Choose the algorithm by setting "ALG=TRIVIUM" or "ALG=TRIVIUM".

2. Append "-D USE_FIRST_CRITERION" to "MACRO=xxx" if the first criterion is to be used, while the second criterion is used by default.

3. Setting "-D WORKER_NUM=xx -D GUROBI_NUM=xx" according to your computer.

4. Please check your version of Gurobi, and modify "-lgurobi91" if necessary.


### To search for valuable cubes

1. Revise the initialization of "vector<wind_t> candidates" in "search.cpp".

2. The input of search_xx.run is (tar_round, preset_cube_index, key_index).

For example, if we want to search for valuable cubes to attack 840-round Trivium theoretically, we set "candidates" as {0, 1, ..., 79} and choose a candidate secret variable, e.g. k_1. Then the following command will search for all valuable cubes whose dimension is 78:

`./search_TRIVIUM.run 840 -1 1`

If we want to search valuable cubes whose dimension is 77, we can choose a preset cube index, e.g. 20, and run the following command:

`./search_TRIVIUM.run 840 20 1`

This command will search for all valuable cubes whose dimension is 77 and indexes are from {0, 1, ..., 19, 21, ..., 79}.


### To retrieve the superpolys

1. Revise the initialization of "vector<wind_t> candidates" in "retrieve.cpp".

2. The input of retrieve_xx.run is (tar_round, preset_cube_index, key_index, cube_index1, cube_index2).

If we found a valuable cube for 840-round Trivum whose indexes are {0, 1, ..., 79} / {10, 26}, to retrieve its superpoly, we run the following conmmand:

`./retrieve_TRIVIUM.run 840 -1 -1 10 26`

If we only want to find out the monomials that contain k2 in the above superpoly, we run the following command:

`./retrieve_TRIVIUM.run 840 -1 2 10 26`

If we want to recover the superpoly of a cube whose indexes are {0, 1, ..., 79} / {10, 20, 26}, the following command could be used:

`./retrieve_TRIVIUM.run 840 20 -1 10 26`

The retrieved superpolys are expressed as lists of monomials. The following data 

` 27  3`
` 26  3`
` 25 26  3`
` 24 25  3`

means the superpoly constains k_27, k_26, k_25 * k_26, k_24 * k_25. The number "3" is the number of solutions this monomial relates to.
