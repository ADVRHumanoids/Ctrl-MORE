#include <locomotion/utils/signalFilters/SignalFilters.hpp>
namespace Locomotion {

SignalFilters::~SignalFilters(){

}
SignalFilters::SignalFilters(){
//Default constructure. initialize all filter variables in 0
    m_Ncoeff=6; 					// Number of coefficients
    int i=0;
    for (i=0;i<MAX_FILTER_LENGTH;i++){ //Init filter in 0
        m_x[i]=0;
        m_y[i]=0;
        m_a[i]=0;
        m_b[i]=0;
    }
}
void SignalFilters::clear_filter(){
//clear the filter as constructure does
    int i=0;
    for (i=0;i<MAX_FILTER_LENGTH;i++){ //init filter in 0
        m_x[i]=0;
        m_y[i]=0;
        m_a[i]=0;
        m_b[i]=0;
    }
}

void SignalFilters::initialValue(double initialState){
//clear the filter as constructure does
    int i=0;
    for (i=0;i<MAX_FILTER_LENGTH;i++){ //init filter in 0
        m_x[i]=initialState;
        m_y[i]=initialState;
    }
}
void SignalFilters::least_squares_filter(double T, int N){
//recibe T as sample Time (s)
//N as the order for the filter

    double freq=1/T;  //Define the frequency
    clear_filter();  //prepared the filter

    //accoriding to the order N do
    if (N == 4) {
        m_Ncoeff = 4;
        m_b[0] = -0.3*freq;
        m_b[1] = -0.1*freq;
        m_b[2] =  0.1*freq;
        m_b[3] =  0.3*freq;
    }
    else if (N == 8) {
        m_Ncoeff = 8;
        m_b[0] = -0.0833*freq;
        m_b[1] = -0.0595*freq;
        m_b[2] = -0.0357*freq;
        m_b[3] = -0.0119*freq;
        m_b[4] =  0.0119*freq;
        m_b[5] =  0.0357*freq;
        m_b[6] =  0.0595*freq;
        m_b[7] =  0.0833*freq;
    }
    else { //Fail gracefully
        m_Ncoeff = 2;
        m_b[0] = -freq;
        m_b[1] =  freq;
    }
}

void SignalFilters::moving_average_filter(int N){
// recive the order of the filter through the variable N
    clear_filter();
    int cnt;
    double C; //constant divisor
    if (N > MAX_FILTER_LENGTH)
        N = MAX_FILTER_LENGTH;

    clear_filter();
    m_Ncoeff = N;
    C = 1.0/N;
    for (cnt = 0; cnt < N; cnt ++) {
        m_a[cnt] = C;
    }
}


/** Build a butterworth filter. T is the sample period,
    *  cutoff is the cutoff frequency in hertz, N is the order (1,2,3 or 4)
    */
void SignalFilters::butterworth(double T, double cutoff, int N){
    //recive sample time (s) T , cute off frequency (Hz) cutoff, and the order of the filter
    //Generate the filter coefficients where a[0] is the cero order factor in the numerator
    clear_filter();
    double A;
    if (N>4)
        N=4;
    if (N==0)
        N=1;
    double C=1/tan(M_PI*cutoff*T);
    if (N==1){
        A=1/(1+C);
        m_a[0]=A;
        m_a[1]=A;
        m_b[0]=1;
        m_b[1]=(1-C)*A;
    }
    if (N==2){
        A=1/(1+1.4142135623730950488016887242097*C+pow(C,2));
        m_a[0]=A;
        m_a[1]=2*A;
        m_a[2]=A;

        m_b[0]=1;
        m_b[1]=(2-2*pow(C,2))*A;
        m_b[2]=(1-1.4142135623730950488016887242097*C+pow(C,2))*A;
    }
    if (N==3){
        A=1/(1+2*C+2*pow(C,2)+pow(C,3));
        m_a[0]=A;
        m_a[1]=3*A;
        m_a[2]=3*A;
        m_a[3]=A;

        m_b[0]=1;
        m_b[1]=(3+2*C-2*pow(C,2)-3*pow(C,3))*A;
        m_b[2]=(3-2*C-2*pow(C,2)+3*pow(C,3))*A;
        m_b[3]=(1-2*C+2*pow(C,2)-pow(C,3))*A;
    }
    if (N==4){
        A=1/(1+2.6131259*C+3.4142136*pow(C,2)+2.6131259*pow(C,3)+pow(C,4));
        m_a[0]=A;
        m_a[1]=4*A;
        m_a[2]=6*A;
        m_a[3]=4*A;
        m_a[4]=A;

        m_b[0]=1;
        m_b[1]=(4+2*2.6131259*C-2*2.6131259*pow(C,3)-4*pow(C,4))*A;
        m_b[2]=(6*pow(C,4)-2*3.4142136*pow(C,2)+6)*A;
        m_b[3]=(4-2*2.6131259*C+2*2.6131259*pow(C,3)-4*pow(C,4))*A;
        m_b[4]=(1-2.6131259*C+3.4142136*pow(C,2)-2.6131259*pow(C,3)+pow(C,4))*A;
    }
}

double SignalFilters::applyFilter(double X){
    //assumes the filter was already run so that the coefficients are able
    //recive the new input- retunrn the filter signal
    m_x[4]=m_x[3];
    m_x[3]=m_x[2];
    m_x[2]=m_x[1];
    m_x[1]=m_x[0];
    m_x[0]=X;
    m_y[4]=m_y[3];
    m_y[3]=m_y[2];
    m_y[2]=m_y[1];
    m_y[1]=m_y[0];
    m_y[0]=m_a[0]*m_x[0]+m_x[1]*m_a[1]+m_x[2]*m_a[2]+m_x[3]*m_a[3]+m_x[4]*m_a[4]
                -m_y[1]*m_b[1]-m_y[2]*m_b[2]-m_y[3]*m_b[3]-m_y[4]*m_b[4];
    return m_y[0];
}
/** Build a butterworth differentiator. T is the sample period,
    *  cutoff is the cutoff frequency in hertz, N is the order (1,2 or 3)
    */


void SignalFilters::butterDifferentiator(double T, double cutoff, int N){
    //recive sample time T, cute off frequency cutoff, and the order of the filter
    //Generate the filter coefficients where a[0] is the cero order factor in the numerator
    clear_filter();
    double C =1/tan(M_PI*cutoff*T);
    double w=2*C/T;
    int i;
    for (i=0;i<MAX_FILTER_LENGTH;i++) //init filter in 0
        m_x[i]=1;

    if (N==1){

        m_a[0]= 1;
        m_a[1]= -1;
        m_b[0]=T*(1+C)/2;
        m_b[1]=T*(1-C)/2;
    }
    if(N==2){

        m_a[0]= 1;
        m_a[1]= 0;
        m_a[2]= -1;
        m_b[0]= T/2*(1+1.414213562373095*C+pow(C,2));
        m_b[1]=T/2*(2-2*pow(C,2));
        m_b[2]=T/2*(1-1.414213562373095*C+pow(C,2));
    }
    if (N==3){

        m_a[0]=1;   //2*(T/2) multiplicative factor
        m_a[1]=1;
        m_a[2]=-1;
        m_a[3]=-1;
        m_b[0]=T/2*(1+2*C+2*pow(C,2)+pow(C,3));
        m_b[1]=T/2*(3+2*C-2*pow(C,2)-3*pow(C,3));
        m_b[2]=T/2*(3-2*C-2*pow(C,2)+3*pow(C,3));
        m_b[3]=T/2*(1-2*C+2*pow(C,2)-pow(C,3));
    }
    if (N==4){
        m_a[0]=1.5;  //3*(T/2) multiplicative factor
        m_a[1]=1;   //2*(T/2) multiplicative factor
        m_a[2]=0;
        m_a[3]=-1;
        m_a[4]=-1.5;
        m_b[0]=T/2*(1+2.6131259*C+3.4142136*pow(C,2)+2.6131259*pow(C,3)+pow(C,4));
        m_b[1]=T/2*(4+2*2.6131259*C-2*2.6131259*pow(C,3)-4*pow(C,4));
        m_b[2]=T/2*(6*pow(C,4)-2*3.4142136*pow(C,2)+6);
        m_b[3]=T/2*(4-2*2.6131259*C+2*2.6131259*pow(C,3)-4*pow(C,4));
        m_b[4]=T/2*(1-2.6131259*C+3.4142136*pow(C,2)-2.6131259*pow(C,3)+pow(C,4));
    }
}

double SignalFilters::differentiator(double X){
    //assumes the filter was already run so that the coefficients are able
    //recive the new input- retunrn the filter signal
    m_x[4]=m_x[3];
    m_x[3]=m_x[2];
    m_x[2]=m_x[1];
    m_x[1]=m_x[0];
    m_x[0]=X;
    m_y[4]=m_y[3];
    m_y[3]=m_y[2];
    m_y[2]=m_y[1];
    m_y[1]=m_y[0];
    m_y[0]=(m_a[0]*m_x[0]+m_x[1]*m_a[1]+m_x[2]*m_a[2]+m_x[3]*m_a[3]+m_x[4]*m_a[4]
            -m_y[1]*m_b[1]-m_y[2]*m_b[2]-m_y[3]*m_b[3]-m_y[4]*m_b[4])/m_b[0];
    return m_y[0];
}

}
