

extern long int read7802(int channel);

void overload(void);
void calculation(int channel);
long int avg(int channel);
void calibration(int channel);
void accuracy(void);

union Data
{
long int value;
char ch[4];
float cf1;
}cap,multiplication_factor,calibration_facotr,BaseCount;


char flag=0,neg_f=0,dp_flag=0,key,dp=0,acc=0,atc=0,LD=0,p0=0;
int n=0;
float diff,wt_f=0;
long int CurrentCount,loadd,weight,count,editt,c;


//LD(Locking Division)=1-9
//                    = if weight is fluctuating because of external environment you can avoid it by using LD
//                    =acc*LD=2*9=18gm(weight<18gms should not be displayed on display )
//Acc(Accuracy)		  =1-50
//                    =if you put weight of 10 kg on scale. scale should show weight in equal parts depend on accuary setting
//DP(Decimal Point)   =1-5
//MF(Multiplication Factor)=weight*MF
