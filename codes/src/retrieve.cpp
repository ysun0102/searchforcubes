

#include "cube.hpp"

using namespace std;





struct input_t {

    char keys[KEY_SIZE];
    char cubes[IV_SIZE];

    wind_t tar_round;
    wind_t preset_cube_index;
        
    wind_t key_index;

    wind_t cube_index1;
    wind_t cube_index2;

};


struct parameters_t {
    input_t *p_in;
    dyn_task_t *dyn_task;  
    wind_t thread_idx;
};





pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
#define DATA_LOCK() {do {pthread_mutex_lock(&data_mutex); } while (0);}
#define DATA_UNLOCK() {do {pthread_mutex_unlock(&data_mutex); } while (0);}





bool cube_retrieve_limited(task_t &tk, input_t *p_in) {


    GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_LogToConsole, 0);
    env.set(GRB_IntParam_Threads, GUROBI_NUM);
    env.set(GRB_IntParam_PoolSearchMode, 2); 
    env.set(GRB_IntParam_MIPFocus, 3);
    env.set(GRB_IntParam_PoolSolutions, MAX_SOLNUM);
    GRBModel model = GRBModel(env);

    vector<GRBVar> S(ALL_STATE_SIZE);
    for (wind_t i = 0; i < ALL_STATE_SIZE; i++) {
        S[i] = model.addVar(0, 1, 0, GRB_BINARY);
    }

    INIT_SETTING()




    // propagates the monomial
    vector<GRBVar> works = S;
    for (wind_t r = 0; r < tk.todo_round; r++) {
        round_update(model, works, r);
    }


    // output function of kreyvium
    for (wind_t i = 0; i < ALL_STATE_SIZE; i++) {
        if (LAST_HAS_BIT(tk.last, i)) {
            model.addConstr(works[i] == 1);
        } else {
            model.addConstr(works[i] == 0);
        }
    }


    model.setObjective(key_deg, GRB_MAXIMIZE);


    // STAGE 1
    model.set(GRB_DoubleParam_NodeLimit, RETRIEVE_STAGE_1_NODE);
    model.optimize();

    if (model.get(GRB_IntAttr_Status) == GRB_NODE_LIMIT) {
        // stage 1 fails
        if (model.get(GRB_DoubleAttr_PoolObjBound) >= RETRIEVE_STAGE_1_OBJ) {
            return false;
        } else {
            // STAGE 2
            model.set(GRB_DoubleParam_NodeLimit, RETRIEVE_STAGE_2_NODE);
            model.optimize();

            if (model.get(GRB_IntAttr_Status) == GRB_NODE_LIMIT) {
                // stage 2 fails
                if (model.get(GRB_DoubleAttr_PoolObjBound) >= RETRIEVE_STAGE_2_OBJ) {
                    return false;
                } else {
                    // STAGE 3
                    model.set(GRB_DoubleParam_NodeLimit, RETRIEVE_STAGE_3_NODE);
                    model.optimize();

                    if (model.get(GRB_IntAttr_Status) == GRB_NODE_LIMIT) {
                        // stage 3 fails
                        if (model.get(GRB_DoubleAttr_PoolObjBound) >= RETRIEVE_STAGE_3_OBJ) {
                            return false;
                        } else {
                            // last chance
                            model.set(GRB_DoubleParam_NodeLimit, RETRIEVE_NODE_LIMIT);
                            model.optimize();

                            if (model.get(GRB_IntAttr_Status) == GRB_NODE_LIMIT) {
                                return false;
                            } // stage final 
                        }
                    } // stage 3 sucess
                }
            } // stage 2 sucess
        }
    } // stage 1 sucess





    



    // model has solved.
    wind_t solCount = model.get(GRB_IntAttr_SolCount);

    if (solCount > 0) {

        printf("[%d]:          +Solutions: %d\n", tk.thread_idx, solCount);

        map_t counterMap;
        sol_t start;
        for (wind_t i = 0; i < solCount; i++) {
            model.set(GRB_IntParam_SolutionNumber, i);
            for (wind_t j = 0; j < KEY_SIZE; j++) {
                if (round(x[j].get(GRB_DoubleAttr_Xn)) == 1) {
                    start[j] = 1;
                } else {
                    start[j] = 0;
                }
            }

            for (wind_t j = 0; j < IV_SIZE; j++ ) {
                if (round(v[j].get(GRB_DoubleAttr_Xn)) == 1) {
                    start[KEY_SIZE + j] = 1;
                } else {
                    start[KEY_SIZE + j] = 0;
                }
            }

            counterMap[start]++;
        }

        PRINTF_STAMP("[%d]: Task %d \t\t ============ found %ld solutions\n\n", tk.thread_idx, tk.task_id, counterMap.size());

        // write to file
        string str = "retrieve_round" + to_string(p_in->tar_round) + "_prev" + to_string(p_in->preset_cube_index) + "_k" + to_string(p_in->key_index) + "_v" + to_string(p_in->cube_index1) + "_v" + to_string(p_in->cube_index2) + "/" + to_string(tk.cur_depth);

        for (wind_t i = 0; i <= tk.cur_depth; i++) {
            str += "_" + to_string(tk.branch_index[i]);
        }

        FILE *out = fopen(str.c_str(), "w");
        wind_t cc = 0;
        for (auto it: counterMap) {
            printf("\tsol %3d:", cc++);
            for (int j = 0; j < KEY_SIZE; j++) {
                if (it.first[j] == 1) {
                    printf(" %d", j);
                    fprintf(out, " %d", j);
                }
            }
            printf(" [%d]\n", it.second);
            fprintf(out, " [%d]\n", it.second);
        }

        fclose(out);

    } // end of solcount > 0

    return true;

}









void cube_retrieve(task_t &tk, input_t *p_in, dyn_task_t *dyn_task) {

    wind_t todo_round = tk.todo_round;
    wind_t depth = tk.cur_depth;
    wind_t mon_i = tk.branch_index[depth];    
    wind_t mon_size = tk.all_size[depth];
    
    wind_t task_id = tk.task_id;
    wind_t thread_idx = tk.thread_idx;

    // branch information    
    PRINTF_STAMP("[%d]: [R] Task %d, Todo %ld, Done %d, Round %d\n", thread_idx, task_id, dyn_task->task_array.size(), dyn_task->n_done, todo_round);

    printf("\tProgress:");
    for (wind_t i = 0; i <= depth; i++) {
        printf(" D%d: %d/%d,", i, tk.branch_index[i] + 1, tk.all_size[i]);
    }
    printf("\n");


    if (!cube_retrieve_limited(tk, p_in)) {

        // expand
        poly_t sub_monomials;
        wind_t next_todo_round = todo_round - expand_polynomial_trial(tk.last, sub_monomials);

        if (depth + 1 >= MAX_DEPTH) {
            PRINTF_STAMP("This should be impossible, since depth reach the max!!!\n");
            exit(1);
        }


        // generate new tasks
        for (wind_t i = 0; i < sub_monomials.size(); i++) {

            task_t next_tk = tk;

            next_tk.branch_index[depth + 1] = i;
            next_tk.all_size[depth + 1] = sub_monomials.size();

            next_tk.cur_depth = depth + 1;
            next_tk.todo_round = next_todo_round;

            next_tk.task_id = 0;
            next_tk.thread_idx = 0;

            last_copy(next_tk.last, sub_monomials[i]);

            DATA_LOCK()
            dyn_task->task_array.push_back(next_tk);
            DATA_UNLOCK()
        }
        
        PRINTF_STAMP("[%d]: Task %d time out, expand to %ld sub-branches\n\n", thread_idx, task_id, sub_monomials.size());
    }

    
}






void *get_task(void *paraall) {

    parameters_t *par = (parameters_t *) paraall;
    dyn_task_t *dyn = par->dyn_task;

    task_t tk;
    bool i_am_working = true;

    while (true) {
        DATA_LOCK()
        if (dyn->task_array.size() == 0) {
            if (dyn->working_threads == 0) {               
                // all works are done
                DATA_UNLOCK()                
                return NULL; 
            } else if (dyn->working_threads == 1 && i_am_working) {
                // no worker working, only I am wokring
                dyn->working_threads--;
                i_am_working = false;
                DATA_UNLOCK()
                return NULL; 
            } else {
                // still has workers
                if (i_am_working) {
                    dyn->working_threads--;
                    i_am_working = false;
                }
                DATA_UNLOCK()
                sleep(SLEEP_TIME); // in seconds.
                continue;
            }
        }

        if (!i_am_working) {
            dyn->working_threads++;
            i_am_working = true;
        }

        // assign task
        tk = dyn->task_array[dyn->task_array.size() - 1];
        tk.task_id = dyn->task_array.size() - 1;
        dyn->task_array.pop_back();
        tk.thread_idx = par->thread_idx;
        DATA_UNLOCK()

        cube_retrieve(tk, par->p_in, dyn);

        PRINTF_STAMP("[%d]: Task %d, Todo %ld, Done %d, Workers %d, is done\n\n", tk.thread_idx, tk.task_id, dyn->task_array.size(), dyn->n_done, dyn->working_threads);

        DATA_LOCK()
        dyn->n_done += 1;       
        DATA_UNLOCK()
    }

}














int main(int argc, const char * argv[]) {

    //parse input.
    if (argc != 6) {
        printf("Please input tar_round, preset_cube_index, key_index, cube_index1, cube_index2, and try agian.\n");
        exit(1);
    }


    wind_t tar_round = atoi(argv[1]);    
    wind_t preset_cube_index = atoi(argv[2]);
    wind_t key_index = atoi(argv[3]);

    wind_t cube_index1 = atoi(argv[4]);
    wind_t cube_index2 = atoi(argv[5]);

    

    printf("++++++++++++++++++++++++++++ parameter +++++++++++++++++++++++++++++++++\n");
    printf("Target round: %d, key index: %d\n", tar_round, key_index);
    printf("Preset cube index: %d\n\n", preset_cube_index);

    printf("Worker number: %d\n", WORKER_NUM);
    printf("Gurobi threads: %d\n", GUROBI_NUM);

    cout << getCurrentSystemTime() << endl << endl;


    char filename[255];
    sprintf(filename, "retrieve_round%d_prev%d_k%d_v%d_v%d", tar_round, preset_cube_index, key_index, cube_index1, cube_index2);
    mkdir(filename, S_IRWXU);




    PRINTF_STAMP("Step 1: Initialization\n\n");

    input_t *p_in = (input_t *) al_calloc(1, sizeof(input_t));

    memset(p_in->keys, S_NOTSURE, KEY_SIZE);
    if (key_index >= 0) {
        p_in->keys[key_index] = S_1;    
    }
    




    // S_b: 44->search 42 or 41
    // vector<wind_t> candidates = {0,2,4,6,8,10,11,12,14,15,16,18,19,20,21,22,23,25,27,29,30,32,34,36,37,39,41,43,45,47,50,53,54,55,60,62,64,69,71,72,75,79, 57,76};

    // for 840, size = 80
    vector<wind_t> candidates = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79};



    memset(p_in->cubes, S_0, IV_SIZE);
    for (wind_t i = 0; i < candidates.size(); i++) {
        p_in->cubes[candidates[i]] = S_1;
    }


    if (preset_cube_index >= 0) {
        // 3cubes
        bool flag = false;
        for (auto cd : candidates) {
            if (cd == preset_cube_index) {
                flag = true;
                break;
            }
        }
        if (!flag) {
            printf("preset cube is not in candidates\n");
            exit(1);
        }
        
        p_in->cubes[preset_cube_index] = S_0;
    }

    p_in->cubes[cube_index1] = S_0;
    p_in->cubes[cube_index2] = S_0;



    p_in->tar_round = tar_round;
    p_in->preset_cube_index = preset_cube_index;
    p_in->key_index = key_index;

    p_in->cube_index1 = cube_index1;
    p_in->cube_index2 = cube_index2;






    PRINTF_STAMP("Step 2: Expanding Polynomial\n\n");

    poly_t monomials;
    expand_polynomial_first(monomials); 

    PRINTF_STAMP("Polynomial expansion has been done\n");


    






    PRINTF_STAMP("\tStep 3: Retrieve in parallel...\n\n");

    dyn_task_t dyn_body;
    dyn_body.task_array.clear();   

    for (wind_t i = 0; i < monomials.size(); i++) {
        task_t cur_task;

        cur_task.branch_index[0] = i;
        cur_task.all_size[0] = monomials.size();

        cur_task.cur_depth = 0;
        cur_task.todo_round = tar_round - FIRST_EXPAND;

        last_copy(cur_task.last, monomials[i]);

        dyn_body.task_array.push_back(cur_task);

    }
    dyn_body.n_done = 0;

    dyn_body.working_threads = WORKER_NUM;


    // initialize
    pthread_t threads[WORKER_NUM];
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    parameters_t para[WORKER_NUM];
    
    for (wind_t th = 0; th < WORKER_NUM; th++) {
        para[th].p_in = p_in;
        para[th].dyn_task = &dyn_body;
        para[th].thread_idx = th;

        pthread_create(threads + th, &attr, get_task, (void *) (para + th));
    }

    for (wind_t th = 0; th < WORKER_NUM; th++) {
        pthread_join(threads[th], NULL);
    }







    PRINTF_STAMP("\tStep 3: Save information...\n\n");

    sprintf(filename, "retrieve_round%d_prev%d_k%d_v%d_v%d_done", tar_round, preset_cube_index, key_index, cube_index1, cube_index2);
    FILE *out = fopen(filename, "w");
    if (out) {
        fclose(out);
    }
    

    al_free(p_in);

    PRINTF_STAMP("\tCompution is finished\n\n");

    cout << getCurrentSystemTime() << endl << endl;

}

