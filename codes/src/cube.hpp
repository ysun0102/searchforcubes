//
//  Created by Yao Sun on 2021/11/8.
//

#ifndef CUBE_HPP
#define CUBE_HPP





// ********************* inlcudes ************************

// standard lib
#include <limits.h>
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdio>
#include<bitset>
#include <vector>
#include <map>
#include <cmath>
#include <fstream>
#include<chrono>
#include<string.h>
#include "sys/time.h"

// pthread
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>


// gubori
#include "gurobi_c++.h"

using namespace std;


// ********************** controllors **********************


//! controllors

// model basic
#ifndef WORKER_NUM
#define WORKER_NUM 12
#endif

#ifndef GUROBI_NUM
#define GUROBI_NUM 8
#endif









// ********************** constants **********************

#define SLEEP_TIME 5


#define MINIMAL_EXPAND 2
#define BRANCH_LIMIT 64
#define MAX_DEPTH 64

#define MAX_SOLNUM 20000000

#define S_0 0
#define S_1 1
#define S_NOTSURE 3


// ! type redefinition
typedef int                         wind_t; 
typedef unsigned int                last_t; 





#ifdef ALG_TRIVIUM

#include "trivium.def"

#endif

#ifdef ALG_KREYVIUM

#include "kreyvium.def"

#endif





struct task_t {
    wind_t branch_index[MAX_DEPTH];
    wind_t all_size[MAX_DEPTH];

    wind_t cur_depth;
    wind_t todo_round;

    wind_t task_id;
    wind_t thread_idx;

    last_t last[LAST_WORD_NUM];

};


struct dyn_task_t {
    vector<task_t> task_array;
    wind_t n_done;
    wind_t working_threads;
};










// ********************** functions **********************




//! useful functions
#define AL_MAX(a, b) ((a) > (b) ? (a) : (b))
#define AL_MIN(a, b) ((a) < (b) ? (a) : (b))


#define CHECK_MEM(p) {if ((p) == NULL) { printf("run out of memory!!\n"); abort(); }}



#define BIN2(n) ( ( (n) * ((n)-1) ) / 2 )

#define ALLCUBE ( BIN2(IV_SIZE) )

// idx1 < idx2
#define CUBE_MAP(idx1, idx2) ( BIN2(idx2) + (idx1) )

#define CUBE_STATUS_HIGHER -1


#define LAST_SHIFT 5
#define LAST_BIT_LEN 32

#define LAST_SET(last, pos) {(last)[(pos) >> LAST_SHIFT] |= (last_t)(1) << ((pos) % LAST_BIT_LEN);}
#define LAST_HAS_BIT(last, pos) (((last)[(pos) >> LAST_SHIFT] >> ((pos) % LAST_BIT_LEN)) & 1)











//! for memory
static inline void *al_calloc(size_t n, size_t size) {
    void *p = calloc(AL_MAX(n, 1),  size);
    CHECK_MEM(p)
    return p;
}


static inline void *al_free(void *p) {
    if (p)
        free(p);
    return NULL;
}







/* convert timeval to miliseconds */
#define TIMEVAL2F(stamp) ((stamp).tv_sec * 1000.0 + (stamp).tv_usec / 1000.0)

/* get timestamp to the precision of miliseconds since the program starts */
double get_timestamp() {
    static double __init_stamp = -1;
    static struct timeval __cur_time;
    if (-1 == __init_stamp) {
        gettimeofday(&__cur_time, NULL);
        __init_stamp = TIMEVAL2F(__cur_time);
    }
    
    gettimeofday(&__cur_time, NULL);
    return ((TIMEVAL2F(__cur_time) - __init_stamp) / 1000.0);
}

/* print msg with timestamp */
#define PRINTF_STAMP(format, ...) \
do { \
    flockfile(stdout); \
    printf("%12.2f - ", get_timestamp()); \
    printf(format, ##__VA_ARGS__); \
    fflush(stdout); \
    funlockfile(stdout); \
} while(0)


string getCurrentSystemTime() {
    auto tt = chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm* ptm = localtime(&tt);
    char date[60] = { 0 };
    sprintf(date, "%d-%02d-%02d-%02d:%02d:%02d", (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday, (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
    return string(date);
}



inline void last_copy(last_t *last, mon_t &mon) {
    memset(last, 0, sizeof(last_t) * LAST_WORD_NUM);
    for (wind_t i = 0; i < ALL_STATE_SIZE; i++) {
        if (mon[i] == 1) {
            LAST_SET(last, i)
        }
    }

}









void _expand_polynomial(last_t *last, map<mon_t, wind_t, cmp_all> &counterMap, wind_t stride) {

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

    // propagate the monomials
    vector<GRBVar> works = S;
    for (wind_t r = 0; r < stride; r++) {
        round_update(model, works, r);
    }

    // output function of kreyvium
    for (wind_t i = 0; i < ALL_STATE_SIZE; i++) {
        if (LAST_HAS_BIT(last, i)) {
            model.addConstr(works[i] == 1);
        } else {
            model.addConstr(works[i] == 0);
        }
    }

    GRBLinExpr obj = 0;
    for (wind_t i = 0; i < ALL_STATE_SIZE; i++) {
        obj += S[i];
    }
    model.setObjective(obj, GRB_MAXIMIZE);

    model.optimize();

    if(model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        double time = model.get(GRB_DoubleAttr_Runtime);
        wind_t solCount = model.get(GRB_IntAttr_SolCount);
        mon_t start;
        for (wind_t i = 0; i < solCount; i++) {
            model.set(GRB_IntParam_SolutionNumber, i);
            for (wind_t j = 0; j < ALL_STATE_SIZE; j++) {
                if (round(S[j].get(GRB_DoubleAttr_Xn)) == 1) {
                    start[j] = 1;
                } else {
                    start[j] = 0;
                }
            }
            counterMap[start]++;
        }
    } else if(model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
        cout << "No monomials " << endl;
        exit(1);     
    } else {
        cout << "Other status " << GRB_IntAttr_Status <<  endl;
        exit(1);
    }
    cout << endl;
}







wind_t expand_polynomial_trial(last_t *last, poly_t &res) {
    wind_t stride = MINIMAL_EXPAND;
    map<mon_t, wind_t, cmp_all> counterMap;

    _expand_polynomial(last, counterMap, stride);

    for (auto it : counterMap) {
        if (it.second % 2 == 1) {
            res.push_back(it.first);
        }
    }

    while (res.size() < BRANCH_LIMIT) {
        stride += 2;
        counterMap.clear();
        _expand_polynomial(last, counterMap, stride);

        res.clear();
        for (auto it : counterMap) {
            if (it.second % 2 == 1) {
                res.push_back(it.first);
            }
        }
    }

    return stride;
} 












#endif /* CUBE_HPP */
