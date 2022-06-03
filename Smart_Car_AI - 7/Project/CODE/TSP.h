#ifndef CODE_TSP_H
#define CODE_TSP_H

#include "common.h"

#define Max_gen 200 //最大进化次数
#define Size_pop 100 //种群数目
#define pcross 0.6 //交叉概率
#define pmutation 0.1 //变异概率
#define lenchrom 14 // 染色体长度(这里即为城市个数)

extern int chrom[Size_pop][lenchrom];
extern int best_result[lenchrom]; // 最佳路线
extern double min_distance; // 最短路径长度

// 函数声明
void init(void); // 种群初始化函数
double * min(double *); // 计算距离数组的最小值
double path_len(int *); // 计算某一个方案的路径长度，适应度函数为路线长度的倒数
void Choice(int [Size_pop][lenchrom]); // 选择操作
void Cross(int [Size_pop][lenchrom]); // 交叉操作
void Mutation(int [Size_pop][lenchrom]); // 变异操作
void Reverse(int [Size_pop][lenchrom]); // 逆转操作

void gen_tsp(void);          //遗传算法求解TSP
#endif