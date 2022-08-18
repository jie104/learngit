//
// Created by yj on 21-2-22.
//

#include "speed_plan.h"


bool SpeedPlan::calc_P(vector<double>& times){



    P = MatrixXd::Zero(m_*n_,m_*n_); // 首先初始化一个60*60的矩阵。
    MatrixXd P0 = MatrixXd::Zero(m_,m_);// 初始化一个6*6的矩阵用于保存每一段的计算。

// 初始化每段时间。


    for(int j=0;j<n_;j++){

        for(int i=3;i<m_;i++){  // 第i行。
            for(int l = 3;l<m_;l++){ // 第l列。
                P0(i,l) = i*(i-1)*(i-2)*l*(l-1)*(l-2)/(i+l-5)*pow(times[j],i+l-5);
            }
        }
        P.block(j*m_,j*m_,m_,m_) = P0;
    }


    //cout<<P<<endl;

    return  true;

}

void SpeedPlan::calc_coff(VectorXd& start,int order,double time){
    start = VectorXd::Zero(m_);
    for(int i=order;i<m_;i++){
        switch (order){
            case 0:{
                start(i) = pow(time,i);
            }break;
            case 1:{
                start(i) = i*pow(time,i-1);
            }break;
            case 2:{
                start(i) = i*(i-1)*pow(time,i-2);
            }break;
            case 3:{
                start(i) = i*(i-1)*(i-2)*pow(time,i-3);
            }break;
            default:
                break;
        }

    }
}


//什么时候计算下一段。当然是刚开始走第一段时就开始从中间点计算下一段。直到计算到终点。
//
// start_status 表示 速度规划的起始点的状态。主要包括s,v,a。
// times.表示每一段路径的运行时间。 每一段都是从0开始。
// dist_from_start.表示除去起点后每个端点的距离值。
// point_max_vel.表示除了起点外，每个端点允许最大速度。最后一段终点速度 可以为0 也可以为max_v. 根据实际情况来使用。
// max_acc 表示允许的最大加速度。
// 计算约束矩阵。
bool SpeedPlan::calc_A(Vector3d start_status,
        vector<double>& times,
        vector<double>& dist_from_start,
        vector<double> point_max_vel,
        double max_acc ){
    int constains_num = 0;
    // 算上所有约束。 A矩阵时一个65*60的矩阵。
    int all_rows = 3+3+ 4*(n_-1)+n_+n_-1+n_-1;
   // cout<<"初步计算的all_colum:"<<all_rows;
    A = MatrixXd::Zero(all_rows,n_*m_); // 首先初始化一个60*60的矩阵。

    U = VectorXd::Zero(all_rows);
    L = VectorXd::Zero(all_rows);


    // 起始条件。 s = 0, v =0,a =0.
    VectorXd start = VectorXd::Zero(m_);
    start(0) = 1;
    A.block(constains_num,0,1,6) = start.transpose();
    U(constains_num) = start_status(0);
    L(constains_num) = start_status(0);// 限制起始的位置
    constains_num++;


    start = VectorXd::Zero(m_);
    for(int i=1;i<m_;i++){
        start(i) = i*pow(0,i-1);
    }
    A.block(constains_num,0,1,6) = start.transpose();
    U(constains_num) = start_status(1);
    L(constains_num) = start_status(1);// 限制起始的速度
    constains_num++;

    start = VectorXd::Zero(m_);
    start(2) = 2;
    A.block(constains_num,0,1,6) = start.transpose();
    U(constains_num) = start_status(2);
    L(constains_num) = start_status(2);// 限制起始的加速度
    constains_num++;

    // 终止条件。 s 可以在最终距离范围内移动。v =vmax,a = 0;
    // 暂定可以在距离左右0.05米范围内移动。
    double dist_offset = 0.05;

    calc_coff(start,0,times[n_-1]);
    A.block(constains_num,(n_-1)*m_,1,6) = start.transpose();
    U(constains_num) = dist_from_start[n_-1]+dist_offset;
    L(constains_num) = dist_from_start[n_-1]-dist_offset;
    constains_num++;

    calc_coff(start,1,times[n_-1]);
    A.block(constains_num,(n_-1)*m_,1,6) = start.transpose();
    U(constains_num) = point_max_vel[point_max_vel.size()-1];;
    constains_num++;

    calc_coff(start,2,times[n_-1]);
    A.block(constains_num,(n_-1)*m_,1,6) = start.transpose();
    constains_num++;



    // 平滑条件。1，前一段时间为Si(Ti) = S(i+1)(0); v 和 a,jerk也是一样的要求。
    for(int j =0;j<n_-1;j++){
        for(int i=0;i<4;i++){
            calc_coff(start,i,times[j]);
            A.block(constains_num,j*m_,1,6) = start.transpose();


            calc_coff(start,i,0);
            start = -1.0*start;
            A.block(constains_num,(j+1)*m_,1,6) = start.transpose();
            constains_num++;
        }
    }
    //距离约束。 每段的距离s在 给定距离的正负dist_offset 之间。距离约束已经产生了一定的单调约束了。
    for(int j=0;j<n_-1;j++){
        calc_coff(start,0,times[j]);
        A.block(constains_num,j*m_,1,6) = start.transpose();
        U(constains_num) = dist_from_start[j]+dist_offset;
        L(constains_num) = dist_from_start[j]-dist_offset;
        constains_num++;
    }

    // 速度约束。起点和终点的速度是限制了的。这里只需要限制中间点即可。

    for(int j=0;j<n_-1;j++){
        calc_coff(start,1,times[j]);
        A.block(constains_num,j*m_,1,6) = start.transpose();
        U(constains_num) = point_max_vel[j];
        L(constains_num) = 0;
        constains_num++;
    }

    // 加速度约束。 起点和终点的加速度也是明确了的。这里只需要限制中间点即可。

    for(int j=0;j<n_-1;j++){
        calc_coff(start,2,times[j]);
        A.block(constains_num,j*m_,1,6) = start.transpose();
        U(constains_num) = max_acc;
        L(constains_num) = -max_acc;
        constains_num++;
    }

   // cout<<"约束的数量："<<constains_num<<endl;
   return false;
}
/**
 ********* 功能：将一个矩阵表示成csc形式的稀疏矩阵
 ********* 参数：M_x、M_nnz、M_i、M_p、M
 ******  返回值：无
 * 需要注意的问题：
 ****** 相关说明：
 */
void SpeedPlan::matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
    M_nnz = 0;
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                M_nnz++;
            }
        }
    }
    *M_x = new c_float[M_nnz];
    int ptr1 = 0;
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                (*M_x)[ptr1] = M(i, j);
                ptr1++;
            }
        }
    }
    int ptr2 = 0;
    *M_i = new c_int[M_nnz];
    *M_p = new c_int[M.cols() + 1];
    (*M_p)[0] = 0;
    for(int j = 0; j < M.cols(); j++) {
        int ptr3 = 0;
        for(int i = 0; i < M.rows(); i++) {
            if(M(i, j) != 0) {
                (*M_i)[ptr2] = i;
                ptr2++;
                ptr3++;
            }
        }
        (*M_p)[j + 1] = (*M_p)[j] + ptr3;
    }
}

/**
 ********* 功能：将一个方阵变成下三角阵，并用csc表示法表示这个矩阵
 ********* 参数：M_x、M_nnz、M_i、M_p、M
 ******  返回值：无
 * 需要注意的问题：
 ****** 相关说明：
 */
int SpeedPlan::upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) {
    if(M.cols() != M.rows()) {
        cout << "The matrix isn't symmetry." << endl << "The program will error!" << endl;
        return 0;
    }
    for(int j = 0; j < M.cols(); j++) {
        for(int i = 0; i < M.rows(); i++) {
            if(i > j) {
                M(i, j) = 0;
            }
        }
    }
    matrixToCsc(M_x, M_nnz, M_i, M_p, M);
    return 1;
}
// 有了矩阵P和 A之后。根据osqp的要求进行优化求解。

bool SpeedPlan::optimize() {


    c_float* P_x;
    c_int   P_nnz;
    c_int*   P_i;
    c_int*   P_p;
    c_float* q;
    c_float* A_x;
    c_int   A_nnz;
    c_int*   A_i;
    c_int*   A_p;
    c_float* l;
    c_float* u;
    c_int n = P.rows();
    c_int m = A.rows();
    upperMatrixToCsc(&P_x, P_nnz, &P_i, &P_p, P);
    matrixToCsc(&A_x, A_nnz, &A_i, &A_p, A);
    q = new c_float[n];
    for(int i=0;i<n;i++){
        q[i] = 0;
    }

    l = new c_float[m];
    u = new c_float[m];

    for(int i = 0; i < m; i++) {
        l[i] = L(i);
        u[i] = U(i);
    }


    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
    OSQPSolution test1;


    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);

    // Setup workspace
    bool exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);

    // 将优化后的结果打印出来。
    coff_.clear();
    for(int i=0;i<n_;i++){
        vector<double> co;
        for(int j=0;j<m_;j++){
            co.push_back(work->solution->x[i*m_+j]);
            //LOG(INFO)<<work->solution->x[i*m_+j]<<",";
        }
        coff_.push_back(co);
    }
   // cout<<"多项式系数的个数："<<coff_.size()<<endl;

    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings)  c_free(settings);

    delete[] P_x;
    delete[] P_i;
    delete[] P_p;
    delete[] q;
    delete[] A_x;
    delete[] A_i;
    delete[] A_p;
    delete[] l;
    delete[] u;

    return false;
}



// 根据路径S 来计算对于的t 。t+控制周期。 带入方程计算目标速度v 和目标加速度a。 然后通过PID计算出实际的目标速度。
// 采用多项式伴随矩阵求特征值的方式来求解 多项式的根。

double SpeedPlan::solve_poly_root(vector<double> coff,double s,double t){

    Matrix<double,5,5> compan;
    compan<<
          0,0,0,0,-(coff[0]-s)/coff[5],
            1,0,0,0,-coff[1]/coff[5],
            0,1,0,0,-coff[2]/coff[5],
            0,0,1,0,-coff[3]/coff[5],
            0,0,0,1,-coff[4]/coff[5];

   // cout<<"伴随矩阵："<<compan<<endl;

    //复数动态矩阵
    Eigen::Matrix<complex<double>, Eigen::Dynamic, Eigen::Dynamic> matrix_eigenvalues;

    matrix_eigenvalues = compan.eigenvalues();
   // LOG(INFO)<<"伴随矩阵的特征根："<< matrix_eigenvalues;

    double calc_t=0;
    for(int i=0;i<m_-1;i++)
    {
        if(matrix_eigenvalues.data()[i].imag() ==0&&matrix_eigenvalues.data()[i].real()>0&& matrix_eigenvalues.data()[i].real()<=t+0.1){
            calc_t = matrix_eigenvalues.data()[i].real();
            //LOG(INFO)<<"当前位置对应的时间："<<calc_t;
        }
    }

    return calc_t;

//    double dist = calc_dist(coff,calc_t);
//    cout<<"当前对应时间 计算出来的位置："<<dist<<endl;
//
//    double vel = calc_vel(coff,calc_t);
//    cout<<"当前对应时间 计算出来的速度："<<vel<<endl;
//
//    vel = calc_vel(coff,calc_t+0.02);
//    cout<<"当前对应时间+0.02 计算出来的速度："<<vel<<endl;
//
//    double a = calc_a(coff,calc_t);
//    cout<<"当前对应时间 计算出来的加速度："<<a<<endl;

}


double SpeedPlan::calc_dist(vector<double> coff,double t){
    double dist =0;
    for(int i=0;i<m_;i++){
        dist += coff[i]*pow(t,i);
    }

    return dist;
}

double SpeedPlan::calc_vel(vector<double> coff,double t){
    double vel =0;
    for(int i=1;i<m_;i++){
        vel += i*coff[i]*pow(t,i-1);
    }

    return vel;
}

double SpeedPlan::calc_a(vector<double> coff,double t){
    double a =0;
    for(int i=2;i<m_;i++){
        a += i*(i-1)*coff[i]*pow(t,i-2);
    }

    return a;
}

// 根据起点和终点的 s，v，a直接求出5次多项式的系数。然后对时间采样，最后根据cost
bool SpeedPlan::calc_analytical_coff(Vector3d start_status,Vector3d end_status,double time,double max_v,double max_a,VectorXd& coff,double& cost){
    Matrix<double,6,6> compan;
    compan<<
          1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,2,0,0,0,
            1,time,pow(time,2),pow(time,3),pow(time,4),pow(time,5),
            0,1,2*pow(time,1),3*pow(time,2),4*pow(time,3),5*pow(time,4),
            0,0,2,2*3*pow(time,1),3*4*pow(time,2),4*5*pow(time,3);
    VectorXd b(6);
    b(0) = start_status(0);
    b(1) = start_status(1);
    b(2) = start_status(2);
    b(3) = end_status(0);
    b(4) = end_status(1);
    b(5) = end_status(2);

    VectorXd x = compan.inverse()*b;
    coff = x;
    // cout<<"x"<<x<<endl;
    //cout<<"time:"<<time<<endl;
    double J = 36*pow(x(3),2)*time+24*6*x(3)*x(4)*pow(time,2)+60*4*x(3)*x(5)*pow(time,3)+24*8*pow(x(4),2)*pow(time,3)+
               2*6*60*x(4)*x(5)*pow(time,4)+720*pow(x(5),2)*pow(time,5);
    //cout<<"J:"<<J<<endl;
    vector<double> coff1;
    for(int i=0;i<6;i++){
        coff1.push_back(coff(i));
    }
    double vel_cost;
    vel_cost = fabs(fabs(calc_vel(coff1,0.03))-end_status(1));

    cost = J;
    //cout<<"cost:"<<cost<<endl;
    if(!check_vel(coff, time, max_v+0.05)){
        return  false;
    }

    if(!check_acc(coff, time, max_a+0.05)){
        return  false;
    }

    return true;
}

// 根据起点和终点的 s，v，a直接求出5次多项式的系数。然后对时间采样，最后根据cost
bool SpeedPlan::calc_analytical_coff1(Vector3d start_status,Vector3d end_status,double time,double max_v,double max_a,VectorXd& coff,double& cost){
    Matrix<double,6,6> compan;
    compan<<
          1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,2,0,0,0,
            1,time,pow(time,2),pow(time,3),pow(time,4),pow(time,5),
            0,1,2*pow(time,1),3*pow(time,2),4*pow(time,3),5*pow(time,4),
            0,0,2,2*3*pow(time,1),3*4*pow(time,2),4*5*pow(time,3);
    VectorXd b(6);
    b(0) = start_status(0);
    b(1) = start_status(1);
    b(2) = start_status(2);
    b(3) = end_status(0);
    b(4) = end_status(1);
    b(5) = end_status(2);

    VectorXd x = compan.inverse()*b;
    coff = x;
    // cout<<"x"<<x<<endl;
    //cout<<"time:"<<time<<endl;
    double J = 36*pow(x(3),2)*time+24*6*x(3)*x(4)*pow(time,2)+60*4*x(3)*x(5)*pow(time,3)+24*8*pow(x(4),2)*pow(time,3)+
               2*6*60*x(4)*x(5)*pow(time,4)+720*pow(x(5),2)*pow(time,5);
    //cout<<"J:"<<J<<endl;
    vector<double> coff1;
    for(int i=0;i<6;i++){
        coff1.push_back(coff(i));
    }
    double vel_cost;
    vel_cost = fabs(fabs(calc_vel(coff1,0.03))-end_status(1));

    cost = J+10.0*time+0.0*vel_cost;
    //cout<<"cost:"<<cost<<endl;
    if(!check_vel(coff, time, max_v+0.05)){
        return  false;
    }

    if(!check_acc(coff, time, max_a+0.05)){
        return  false;
    }

    return true;
}


// 判断速度是否满足要求。
bool SpeedPlan::check_vel(VectorXd coff, double time,double max_v)
{
    // 返回ture 表示曲线满足要求。 返回false 表示曲线不满足要求。
    vector<double> coff1;
    for(int i=0;i<6;i++){
        coff1.push_back(coff(i));
    }
    int count = time/0.1;
    for(int i=0;i<=count;i++){
        if(calc_vel(coff1,i*0.1)>max_v){
           // LOG(INFO)<<"速度检查不合格，时间为:"<<i*0.1<<";速度为："<<calc_vel(coff1,i*0.1)<<";time:"<<time;
            return false;
        }
    }

    return  true;

}
// 判断加速度是否满足要求。
bool SpeedPlan::check_acc(VectorXd coff, double time,double max_a){
    // 返回ture 表示曲线满足要求。 返回false 表示曲线不满足要求。
    vector<double> coff1;
    for(int i=0;i<6;i++){
        coff1.push_back(coff(i));
    }
    int count = time/0.1;
    for(int i=0;i<=count;i++){
        if(fabs(calc_a(coff1,i*0.1))>max_a){
            //LOG(INFO)<<"加速度检查不合格，时间为:"<<i*0.1<<";加速度为："<<calc_a(coff1,i*0.1)<<";time:"<<time;
            return false;
        }
    }

    return  true;
}

void SpeedPlan::calc_opt_analytical_coff(Vector3d start_status,Vector3d end_status,double max_v,double max_a,double& time,VectorXd& coff){
    double time1;
    VectorXd x1;
    vector<VectorXd> coffs1;
    vector<double> costs1;
    vector<double> times1;
    double cost=0;

    // 计算base_time 改一改。
    double v0 = start_status(1);
    double v1 = end_status(1);
    double s = (end_status(0) - start_status(0));

    // 直接减速的终点的距离
    double base_s = fabs(v0*v0-v1*v1)/(2.0*max_a);
    double base_time;
    // if(s>base_s){
    //     double v2 =sqrt((2.0*max_a*s+v0*v0+v1*v1)/2.0);
    //     if(v2>max_v){
    //         base_time = (max_v-v0)/max_a+(max_v-v1)/max_a+(s-(max_v-v0)/max_a*(max_v+v0)/2-(max_v-v1)/max_a*(max_v+v1)/2)/max_v;
    //     }
    //     else{
    //         base_time = (2.0*v2-v1-v0)/max_a-1;
    //     }
         
    // }
    // else{
    //     base_time = 2.0*s/(v0+v1)-1;
    // }   

    if(v0!=0){
        base_time = 2.0*s/(v0+v1);
    }
    else{
         double v2 =sqrt((2.0*max_a*s)/2.0);
        if(v2>max_v){
            base_time = (max_v-v0)/max_a+(max_v-v1)/max_a+(s-(max_v-v0)/max_a*(max_v+v0)/2-(max_v-v1)/max_a*(max_v+v1)/2)/max_v;
        }
        else{
            base_time = (2.0*v2-v1-v0)/max_a;
        }
    }
    base_time = time;

    if(base_time<0){
        base_time = 0.025;
    }
    for(int i=0;i<70;i++){
        time1 = 0.05*i+base_time;
        //LOG(INFO)<<"TEST2";
        bool success =  calc_analytical_coff( start_status, end_status, time1,max_v,max_a,x1,cost);
        if(success){
            coffs1.push_back(x1);
            costs1.push_back(cost+2.5*max_v*(time1-base_time));
            times1.push_back(time1);
        }
    }
    double cost_max = 100000;
    int min_id = -1;
    for(int i=0;i<costs1.size();i++){
        if(costs1[i]<cost_max){
            cost_max = costs1[i];
            min_id = i;
        }
    }
    if(min_id<0){
        LOG(INFO)<<"没有找到合适的值！！！！";
        return;
    }
    //LOG(INFO)<<"min_id:"<<min_id<<";cost_max:"<<cost_max<<";time:"<<times1[min_id]<<";base_time:"<<base_time;
    //LOG(INFO)<<"S:"<<s;
    //cout<<"coff:"<<endl;
    //cout<<coffs1[min_id]<<endl;
    coff = coffs1[min_id];
    time = times1[min_id];
}



void SpeedPlan::calc_opt_analytical_coff1(Vector3d start_status,Vector3d end_status,double max_v,double max_a,double& time,VectorXd& coff){
    double time1;
    VectorXd x1;
    vector<VectorXd> coffs1;
    vector<double> costs1;
    vector<double> times1;
    double cost=0;

    // 计算base_time 改一改。
    double v0 = start_status(1);
    double v1 = end_status(1);
    double s = (end_status(0) - start_status(0));
    double base_time = time;
    for(int i=0;i<70;i++){
        time1 = 0.01*i+base_time;
        //LOG(INFO)<<"TEST2";
        bool success =  calc_analytical_coff1( start_status, end_status, time1,max_v,max_a,x1,cost);
        if(success){
            coffs1.push_back(x1);
            costs1.push_back(cost);
            times1.push_back(time1);
        }
    }
    double cost_max = 100000;
    int min_id = -1;
    for(int i=0;i<costs1.size();i++){
        if(costs1[i]<cost_max){
            cost_max = costs1[i];
            min_id = i;
        }
    }
    if(min_id<0){
        LOG(INFO)<<"没有找到合适的值！！！！";
        return;
    }
    //LOG(INFO)<<"min_id:"<<min_id<<";cost_max:"<<cost_max<<";time:"<<times1[min_id];
    //cout<<"coff:"<<endl;
    //cout<<coffs1[min_id]<<endl;
    coff = coffs1[min_id];
    time = times1[min_id];
}

// 根据起点速度 终点速度和起点到终点距离 还有最大速度计算 运行时间。
double calc_time(double sv,double ev,double dist,double max_v){
    //
    return 0.0;
}