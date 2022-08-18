//
// Created by yj on 19-11-28.
//

#include "path_follow.h"

void PathFollow::calc_velocity(velocity_plan velocity,double dt,double& current_v)
{
    unsigned char acc_down_flag;
    uint8_t AccDownCorrectFlag;
    float AccDown;
    float dist_offset = 0.005f;
    if ((velocity.remain_dist - 0.01f) < (pow( current_v,2) - pow(velocity.end_v,2)) / 2.0f / velocity.acc_down)
    {
    acc_down_flag = 1;
    }
    else
    {
    acc_down_flag = 0;
    }

    if (acc_down_flag == 0)
    {
        if (fabs( current_v) <velocity.max_v)
        {
         current_v = fabs( current_v) + velocity.acc_up  * dt;
        }
        else
        {

         current_v = fabs( current_v) - velocity.acc_down * dt;
        }
    }

    if (acc_down_flag == 1)
    {
        if (velocity.remain_dist > 0.01)
        {
            AccDown = (pow( current_v,2) - pow(velocity.end_v,2)) / 2.0f / abs(velocity.remain_dist  - abs(current_v) * 0.02f - dist_offset);
            // AccDown = ClipFloat(AccDown, -1, 1); //速度限幅
            if(AccDown>fabs(velocity.acc_down)+0.5f)
            {
                AccDown = velocity.acc_down+0.5f;
            }
            else if(AccDown<-1.0f*fabs(velocity.acc_down)-0.5f)
            {
                AccDown = -1.0f*fabs(velocity.acc_down)-0.5f;
            }

            AccDownCorrectFlag = 1;
        }
        else
        {
            AccDownCorrectFlag = 0;
        }

        if (AccDownCorrectFlag == 1)
        {
            current_v = fabs( current_v) - AccDown * dt;
        }
        else
        {
            current_v = fabs( current_v) - velocity.acc_down * dt;
        }
    }

    if ( current_v < dist_offset)
    {
         current_v = dist_offset;
    }
}





bool PathFollow::process_rotate(double &w, double current_angle, double target_angle, double max_w,
                                    double acc_w, double time) {
    //只在路径开始时执行，主要就是原地旋转 采用之前的方式输出角速度即可。

    double angle_error;
    float AccDown;
    uint8_t acc_down_flag = 0;
    uint8_t AccDownCorrectFlag=0;
    float dist_offset = 0.01f;
    angle_error = target_angle - current_angle;
    angle_error = normalizeYaw(angle_error);
    const double ACC_DOWN_OFFSET = 0.02;
    if (fabs(angle_error) < ((pow(w, 2) / 2.0 / acc_w) + ACC_DOWN_OFFSET)) {
       acc_down_flag = 1;
    } else {
        acc_down_flag = 0;

    }

    if(acc_down_flag==0)
    {
        if (fabs(w) < max_w) {
            w = fabs(w) + acc_w * time;
        } else {
            w = fabs(w) - acc_w * time;
        }
    }

    if (acc_down_flag == 1)
    {
        if (fabs(angle_error) > 0.02)
        {
            AccDown = (pow( w,2)) / 2.0f / abs(fabs(angle_error)  - abs(w) * 0.02f - dist_offset);
            // AccDown = ClipFloat(AccDown, -1, 1); //速度限幅
            if(AccDown>fabs(acc_w)+0.5f)
            {
                AccDown = acc_w+0.5f;
            }
            else if(AccDown<-1.0f*fabs(acc_w)-0.5f)
            {
                AccDown = -1.0f*fabs(acc_w)-0.5f;
            }

            AccDownCorrectFlag = 1;
        }
        else
        {
            AccDownCorrectFlag = 0;
        }

        if (AccDownCorrectFlag == 1)
        {
            w = fabs(w) - AccDown * time;
        }
        else
        {
            w = fabs(w) - acc_w * time;
        }
    }



    const double VELOCITY_THRESHOLD = 0.02;
    if (w < VELOCITY_THRESHOLD) {
        w = VELOCITY_THRESHOLD;
    }

    if (angle_error > 0) {
    } else {
        w = -1.0 * w;
    }

    const double ANGLE_GOAL_TOLERANCE=0.02;
    if (abs(angle_error) < ANGLE_GOAL_TOLERANCE&&fabs(w)<0.03) {
        w = 0;
       // LOG(INFO) << "local_planner:完成原地旋转";
        return true;
    }

    return false;


}


double PathFollow::sr_calc_coeff (sros::core::Pose current_position , unsigned char direction, sros::core::Pose target_position)
{
    double xita_V = 0;
    sros::core::Pose TransTargetPosition;
    if (direction == 0x01)
    {
        xita_V = current_position.yaw();
    }
    else if (direction == 0x00)
    {
        xita_V = current_position.yaw() + 3.14159f;
    }
    //计算导航圆弧的曲率
    //将target_position转换为机器人坐标系的坐标
    TransTargetPosition.x() = (target_position.x() - current_position.x()) * cos(xita_V) + (target_position.y() - current_position.y()) * sin(xita_V);
    TransTargetPosition.y() = -(target_position.x() - current_position.x()) * sin(xita_V) + (target_position.y() - current_position.y()) * cos(xita_V);
    //求出D的平方
    double SquareD = TransTargetPosition.x() * TransTargetPosition.x() + TransTargetPosition.y() * TransTargetPosition.y();
    //将绝对坐标系下的圆心点转化为车体坐标系下的点。只需要求y即可
    //求圆弧的曲率
    double gama = 2.0f * TransTargetPosition.y() / SquareD;
    return gama;
}



void PathFollow::WConfigure_doubleS(double max_w,float max_acc_w,float Jmu,float distance,float vs,float as)
{
    float sa = 0;
	  if(distance < 0.001)
		{
		  distance = 0.001;
		}
  	float se = distance;
  	float ve = 0;
	  float ae = 0;
  	float Ts = 0.03; 
		
    float Vmin = max_w;
	  float Vmax = max_w;
	
	
	  float Amin = -max_acc_w;
	  float Amax = max_acc_w;
	  float Jmin= -Jmu*3.5;
	  float Jmax= Jmu*3.5;

    //���³�ʼ�����滮��
    is_InStopPhase = false;
    is_AccelerationBegin = false;

    DoubleS.sa = sa;
    DoubleS.se = se;
    DoubleS.S = fabs(se - sa);
    DoubleS.vs = vs;
    DoubleS.ve = ve;
    DoubleS.as = as;
    DoubleS.ae = ae;
    DoubleS.Ts = Ts;

    //Vmin, Vmax, Amin, Amax, Jmin, Jmax;
    DoubleS.Vmin = Vmin;
    DoubleS.Vmax = Vmax;
    DoubleS.Amin = Amin;
    DoubleS.Amax = Amax;
    DoubleS.Jmin = Jmin;
    DoubleS.Jmax = Jmax;

    //����ָ���������
    DoubleS._Amin = Amin;
    DoubleS._Amax = Amax;
    DoubleS._Jmin = Jmin;
    DoubleS._Jmax = Jmax;
    if (vs <= Vmax)
		{
        is_AccelerationBegin = true;
		}
}


float PathFollow::WNext_doubleS(float *aa)
{
	  float sa = DoubleS.sa;
    float se = DoubleS.se;
    float S = DoubleS.S;
    float vs = DoubleS.vs;
    float ve = DoubleS.ve;
    float as = DoubleS.as;
    float ae = DoubleS.ae;
    float Ts = DoubleS.Ts;
    float Vmin = DoubleS.Vmin;
    float Vmax = DoubleS.Vmax;
    float Amin = DoubleS.Amin;
    float Amax = DoubleS.Amax;
    float Jmin = DoubleS.Jmin;
    float Jmax = DoubleS.Jmax;
	  
	  //����ָ���������
    float _Amin = DoubleS._Amin;
    float _Amax = DoubleS._Amax;
    float _Jmin = DoubleS._Jmin;
    float _Jmax = DoubleS._Jmax;

		float jk = 0; 
		float Tj2a = 0; 
		float Tj2b = 0; 
		float Td = 0; 
		float hk = 0;
		
    float vk = vs;
		float vk1 = vs;
    float ak = as;
		float ak1 = as;
		float TTj2a = 0;
		float TTj2b = 0;
		
    if (!is_InStopPhase)
		{
			//��δ����ֹͣ�׶�(���ٶ�֮��) - ʽ��(4)(5) �� (8)(9) ֻ��Jmin��Jmax�����ˣ�����ϲ��Ż�
			Tj2a = (_Amin - ak) / _Jmin;
			Tj2b = (ae - _Amin) / _Jmax;
			Td = (ve - vk) / _Amin + Tj2a * (_Amin - ak) / (2 * _Amin) + Tj2b * (_Amin - ae) / (2 * _Amin);
			if (Td - (Tj2a + Tj2b) < 0) 
			{
				float temp = sqrt((_Jmax - _Jmin) * (ak * ak * _Jmax - _Jmin * (ae * ae + 2 * _Jmax * (vk - ve))));
				Tj2a = -ak / _Jmin + temp / (_Jmin * (_Jmin - _Jmax));
				Tj2b = ae / _Jmax + temp / (_Jmax * (_Jmax - _Jmin));
				Td = Tj2a + Tj2b;
			}
			hk = 0.5 * ak * Td * Td +(float) 1.0 / 6.0 *(_Jmin * Tj2a * (3 * Td * Td - 3 * Td * Tj2a + Tj2a * Tj2a) + _Jmax * Tj2b * Tj2b * Tj2b) + Td * vk;
			float rate = 0.8;
			if (hk >= S*rate)
			{
				is_InStopPhase = true;
				int count = 0;
				while (hk >= S*rate && count < 9)
				{
					count++;
					_Jmax = _Jmax + DoubleS._Jmax/3;
					_Jmin = _Jmin + DoubleS._Jmin/3;
					//��δ����ֹͣ�׶�(���ٶ�֮��) - ʽ��(4)(5) �� (8)(9) ֻ��Jmin��Jmax�����ˣ��ϲ��Ż�
					TTj2a = (_Amin - ak) / _Jmin;
					TTj2b = (ae - _Amin) / _Jmax;
					float TTd = (ve - vk) / _Amin + TTj2a * (_Amin - ak) / (2 * _Amin) + TTj2b * (_Amin - ae) / (2 * _Amin);
					if (TTd - (TTj2a + Tj2b) < 0)
					{
							double temp = sqrt((_Jmax - _Jmin) * (ak * ak * _Jmax - _Jmin * (ae * ae + 2 * _Jmax * (vk - ve))));
							TTj2a = -ak / _Jmin + temp / (_Jmin * (_Jmin - _Jmax));
							TTj2b = ae / _Jmax + temp / (_Jmax * (_Jmax - _Jmin));
							TTd = TTj2a + TTj2b;
					}
					hk = 0.5 * ak * TTd * TTd +
							(float)1.0 / 6.0 *
							(_Jmin * TTj2a * (3 * TTd * TTd - 3 * TTd * TTj2a + TTj2a * TTj2a) + _Jmax * TTj2b * TTj2b * TTj2b) +
							TTd * vk;   
				}
				if (hk >= S || _Jmax > 6)
				{				
					//�˻�Ϊ���μӼ���
					if (S > 0.002)
					{
						ak = fabs(ve*ve - vk*vk)/(2*S);
						if (fabs(ak) > 2)
						{
							ak = (ve - vk)/fabs(ve-vk)*Amax; 
						}
						else
						{
							ak = (ve - vk)/fabs(ve-vk)*ak;
						}
					}
					else
					{
						ak = 0;
					}
					vk = vk1 + ak*Ts;
					if (vk < 0.01)
					{
						vk = 0.01;
					}
					*aa = ak;
					return vk;	
				}
			}
    }  
		
		if (is_InStopPhase) 
		{
        //����ֹͣ�׶�
				if (-0.0001 < Tj2a)
            jk = _Jmin;
        else if (0 < Td - Tj2b)
				{
            jk = 0;
				}
        else
            jk = _Jmax;
    } 
		else 
		{
        //δ����ֹͣ�׶�
        float temp1 = vk - ak * ak / (2 * Jmax);
			  float temp2 = vk - ak * ak / (2 * Jmin);
        if (!is_AccelerationBegin) 
				{
					//��ʼ�׶�Ϊ���ٽ׶� - ��ʽ(3)
					if (fabs(temp1 - Vmax)>0.01) 
					{
						if (ak > Amin)
						{
								jk = Jmin;
						}
						else
						{
								jk = 0;
						}
					} 
					else
					{
						if (ak < 0)
						{
								jk = Jmax;
						}
						else
						{
								jk = 0;
						}
					}
        } 
				else 
				{
					//��ʼ�׶�Ϊ���ٽ׶� - ��ʽ(2)
					if (temp2 < Vmax) 
					{
						if (ak < Amax)
						{
								jk = Jmax;
						}
						else
						{
								jk = 0;
						}
					}
					else
					{
						if (ak > 0)
						{
								jk = Jmin;
						}
						else
						{
								jk = 0;
						}
				  }
        }
    }
		ak = ak1 + Ts*jk;
    if (ak > Amax)
    {
			ak = Amax;
    }
    if (ak < Amin)
    {
        //ak = Amin;
    }

		vk = vk1 + Ts / 2 * (ak1 + ak);
		if (vk > Vmax)
    {
        vk = Vmax;
        jk = 0;
        ak = 0;
    }
    if ((vk - 0.01)< -0.001 )
    {
        vk = 0.01;
        jk = 0;
        ak = 0;
    }		
		
    ak1 = ak;
    vk1 = vk; 
		*aa = ak1;
		return vk;
}

