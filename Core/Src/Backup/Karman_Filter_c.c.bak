#include "arm_math.h"
#include <math.h>

typedef struct{

		float q;
		float p;
		float r;
		float k;
		float x;
}Karman;


int checkOverflow_Add(float a, float b, float result){
	if((a>0)&&(b>0)&&(result<0)){
		return 1;
	}else if((a<0)&&(b<0)&&(result>0)){
		return 1;
	}
	return 0;
}

int checkOverflow_Mul(float a, float b, float result){
	if((a>0)&&(b>0)&&(result<0)){
		return 1;
	}
	if((a<0)&&(b<0)&&(result<0)){
		return 1;
	}
	if( (( (a>0)&&(b<0) )||( (a<0)&&(b>0) )) && (result>0) ){
		return 1;
	}
	return 0;
}

int updateKarmanFilter_c(Karman* kr, float m){
	Karman* input = kr;
	float temp;
	kr->p += kr->q;
	if (checkOverflow_Add(input->p,input->q,kr->p)){return 1;}

	temp = kr->p + kr->r;
	if (temp == 0){return 1;}
	kr->k = kr->p/temp;

	temp = (m-kr->x);//m-x
	if (checkOverflow_Add(m,-1*input->x,temp)){return 1;}

	float temp_ = kr->k*temp;//k(m-x)
	if (checkOverflow_Mul(kr->k,temp,temp_)){return 1;}

	kr->x += temp_;//x+k(m-x)
	if (checkOverflow_Add(input->x,temp_,kr->x)){return 1;}

	temp = 1-kr->k;//(1-k)//overflow not possible practically
	temp_ = kr->p;
	kr->p = temp*temp_;//p(1-k)
	return checkOverflow_Mul(temp_,temp,kr->p);
}
int updateKarmanFilter_CMSIS(Karman* kr,float32_t m){
	Karman* input = kr;
	float32_t temp;
	float32_t temp_;

	arm_add_f32(&(kr->p),&(kr->q),&(kr->p),1);
	if (checkOverflow_Add(input->p,input->q,kr->p)){return 1;}

	arm_add_f32(&(kr->p),&(kr->r),&temp,1);
	if (temp==0){return 1;}//divide by zero

	//arm_matrix_instance_f32 temp_M = {1,1,&temp};
	//arm_matrix_instance_f32 temp_MI;
	//arm_mat_inverse_f32(&temp_M, &temp_MI);
	//temp = ((&temp_MI)->pData)[1];
	//arm_scale_f32(&(kr->p),temp,&(kr->k),1);

	//q31_t temp_q31;
	//arm_float_to_q31(&temp,&temp_q31,1);
	//q31_t one_const = 1;
	//int16_t temp_shift;
	//arm_status arm_divide_q31(one_const,temp_q31,&temp_q31,&temp_shift);
	//arm_shift_q31(&temp_q31,(int8_t)&temp_shift,&temp_q31,1);
	//arm_q31_to_float(&temp_q31, &temp, 1);
	//arm_mult_f32(&(kr->p),&temp,&(kr->p),1);
	kr->k = kr->p/temp;

	arm_sub_f32(&m,&(kr->x),&temp,1);//temp = m-x
	if (checkOverflow_Add(m,-1*input->x,temp)){return 1;}


	arm_mult_f32(&temp,&(kr->k),&temp_,1);//temp_ = k(m-x)
	if (checkOverflow_Mul(kr->k,temp,temp_)){return 1;}

	arm_add_f32(&(kr->x),&temp_,&(kr->x),1);
	if (checkOverflow_Add(input->x,-1*temp_,kr->x)){return 1;}

	float32_t num_1 = 1;
	arm_sub_f32(&num_1,&kr->k,&temp,1);//1-k//overflow not possible practically
	temp_ = kr->p;
	arm_mult_f32(&temp_,&temp,&kr->p,1);
	return checkOverflow_Mul(temp,temp_,kr->p);
}
