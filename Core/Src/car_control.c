#include "car_control.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"
#include "ctrl_menu.h"

__IO CarCtrl_State_TypeDef g_car_ctrl_state = CarCtrl_STOP ;

car_config_t g_CarConfig = 
{
	.speed_KP = 5,
	.speed_KI = 0.1 ,
	.speed_KD = 0
};
car_ctrl_t 	g_CarCtrl;

car_plan_t* g_CarPlan_Ptr;

car_plan_t g_CarPlan_Base[] =
{
	{ 55  , { 0 , 0} , 0 , 100 } ,   		// test steer moto
	{ -55  , { 0 , 0} , 0 , 100 } ,  		// test steer moto
	
	{ 16  , { 500 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 16  , { 500 , 500} , 0 , 150 } ,		// run 1.5s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,   // turn right 1.1s 
	
	{ 16  , { 500 , 500} , 0 , 120 } ,		// run 1.2s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 135 } ,		// turn right 1.35s 
	
	{ 16  , { 500 , 500} , 0 , 180 } ,		// run 1.8s with 500mm/s speed straightly
	{ 16  , { 0   , 0  } , 0 , 0 } ,		// stop
};

car_plan_t g_CarPlan_Supper[] =
{ // angle left right speed diastance run_time
  		// test sreer moto */ 
	//-：右  左速度 右速度 避障距离 time
	/* { -8,{ 550 , 500} , 40000 , 3000 } ,  	//THIS LINE ONLY FOR OBSTACLE_AVOIDING stage 1 forward 1.8m
 */


	{ -8,{ 550 , 500} , 40000 , 150 } ,
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 80  , { 200 , 400} , 0 , 150 } ,		// turn left 1.1s 
	
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0 ,  0} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 17  , { 500 , 500} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0   , 0  } , 0 , 0 } ,		// stop  */

};
car_plan_t g_CarPlan_Avoid[] =
{
	{ 80  , { 200 , 400} , 0 , 100  }, // 向右微调转弯
	{ -7,{ 550 , 500} , 20000 , 20 },
    { -80  , { 400 , 200} , 0 , 90  }, //向left微调转弯
	{ -7,{ 550 , 500} , 20000 , 100 },    // 前进1m（根据速度和时间算）
   // 停止
	{ -80  , { 400 , 200} , 0 , 90  },
	{ 80  , { 200 , 400} , 0 , 150  },
	{ -7,{ 550 , 500} , 20000 , 10 },
		{ 0   , {0  , 0  } , 0 , 0   }, 
};










/////////////////////////////////////////////////////////////////////////////////
// Menu control
//
/////////////////////////////////////////////////////////////////////////////////
void CarCtrl_Start( void )
{
	g_car_ctrl_state = CarCtrl_IDLE ;
}

void CarCtrl_Stop( void )
{
	g_car_ctrl_state = CarCtrl_STOP ;
	Ctrl_Menu_Show();
}

void CarCtrl_SpeedUp( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] += 5 ;
}

void CarCtrl_SpeedDown( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] -= 5 ;
}

void CarCtrl_SpeedStop( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = 0 ;
}

void CarCtrl_Forward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Backward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = -1*abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Straight( void )
{
	g_CarCtrl.car_angle = 0 ;
	Steer_Moto_Ctrl( STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Right( void )
{
	g_CarCtrl.car_angle -- ;
	if ( g_CarCtrl.car_angle < -45 ) g_CarCtrl.car_angle = -45 ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Left( void )
{
	g_CarCtrl.car_angle ++ ;
	if ( g_CarCtrl.car_angle > 45 ) g_CarCtrl.car_angle = 45 ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}



/////////////////////////////////////////////////////////////////////////////////

void CarCtrl_Init( void )
{
	car_config_t *p_car_cfg = & g_CarConfig ;
	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );

	g_CarPlan_Ptr = g_CarPlan_Supper ;

}

void CarCtrl_Speed_PID( )
{
	static int32_t last_speed[DRIVE_MOTO_NUM] = {0,0};
	static int32_t speed_intergrade[DRIVE_MOTO_NUM] = {0,0};
	int32_t speed_error[DRIVE_MOTO_NUM];
	int32_t speed_diff[DRIVE_MOTO_NUM];

	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		speed_error[i] = g_CarCtrl.car_speed_set[i] - g_speed_encoder[i].speed ;
		speed_intergrade[i] = speed_intergrade[i] + speed_error[i] ; 
		speed_diff[i] = last_speed[i] - g_speed_encoder[i].speed  ;
		last_speed[i] = g_speed_encoder[i].speed ;		
		g_CarCtrl.moto_drive[i] = speed_error[i]*g_CarConfig.speed_KP +
															speed_intergrade[i] * g_CarConfig.speed_KI +
															speed_diff[i] * g_CarConfig.speed_KD  ;
		
		//Drive_Moto_Ctrl( i , g_CarCtrl.moto_drive[i] );
	}
	
	Drive_Moto_Ctrl( 0 , g_CarCtrl.moto_drive[0] );
	Drive_Moto_Ctrl( 1 , -g_CarCtrl.moto_drive[1] );	
}

/* void CarCtrl_PlanSet( void )
{
	

  
	car_plan_t* car_plan_ptr ;
	
	car_plan_ptr = g_CarPlan_Ptr+g_CarCtrl.run_step ;
	
	/* if ( car_plan_ptr->run_time_set == 0 )
	{
		g_car_ctrl_state = CarCtrl_STOP;
		Steer_Moto_Ctrl(STEER_MOTO_POS ,car_plan_ptr->car_angle_set);
		for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
			Drive_Moto_Ctrl( i , 0);
		memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
		return ;
	} 
	 */

/* 	 if ( car_plan_ptr->run_time_set == 0 )
{
	// 停止状态
	g_car_ctrl_state = CarCtrl_STOP;

	// 舵机居中（可选）
	Steer_Moto_Ctrl(STEER_MOTO_POS , 0);

	// 清空目标速度
	CarCtrl_SpeedStop();  // 你已有的函数：car_speed_set[] = 0

	// 停止 PWM 输出
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		g_CarCtrl.moto_drive[i] = 0;         // 清除 PID 输出
		Drive_Moto_Ctrl( i , 0);             // 硬停
	}

	// 可选：重置控制结构体（否则 PID 会继续工作）
	memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );

	return ;
}

	if ( g_CarCtrl.run_time == 0 )  // load plan 
	{
		g_CarCtrl.run_time ++ ;
		for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
			g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i] ;
		Steer_Moto_Ctrl(STEER_MOTO_POS , car_plan_ptr->car_angle_set );		
	}
	else														// execute plan 
	{
		g_CarCtrl.run_time ++ ;
		if ( g_CarCtrl.run_time == car_plan_ptr->run_time_set || 								// plan over
			   g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set )   // distance too close
		{
			g_CarCtrl.run_time = 0 ;
			g_CarCtrl.run_step ++ ;			
		}
	} 
} */
void CarCtrl_PlanSet( void )
{
	car_plan_t* car_plan_ptr ;
	car_plan_ptr = g_CarPlan_Ptr + g_CarCtrl.run_step;

	if ( car_plan_ptr->run_time_set == 0 )
	{
		// 计划结束，停止
		g_car_ctrl_state = CarCtrl_STOP;
		Steer_Moto_Ctrl(STEER_MOTO_POS , 0);
		CarCtrl_SpeedStop();
		for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		{
			g_CarCtrl.moto_drive[i] = 0;
			Drive_Moto_Ctrl( i , 0 );
		}
		memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
		return;
	}

	if ( g_CarCtrl.run_time == 0 )
	{
		// 初始化当前计划
		g_CarCtrl.run_time++;
		for (int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
			g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i];
		Steer_Moto_Ctrl(STEER_MOTO_POS , car_plan_ptr->car_angle_set);
	}
	else
	{
		// 执行中：检测时间或障碍物
		g_CarCtrl.run_time++;

		if ( g_CarCtrl.run_time >= car_plan_ptr->run_time_set ||
		     g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set )
		{
			// 如果前方太近
			if (g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set &&
				g_CarPlan_Ptr == g_CarPlan_Supper)  // 说明还在主路径中
			{
				// 切换为避障路径（你需要定义它）
				extern car_plan_t g_CarPlan_Avoid[];  // 声明
				g_CarPlan_Ptr = g_CarPlan_Avoid;
				g_CarCtrl.run_step = 0;
				g_CarCtrl.run_time = 0;
			}
			else
			{
				// 正常进入下一步
				g_CarCtrl.run_time = 0;
				g_CarCtrl.run_step++;
			}
		}
	}
}


void CarCtrl_Show( void ) 
{
	static uint8_t  index = 0 ;
	static int32_t  speed[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	static int32_t  pwm[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	uint8_t 				buf[17];
	
	for (int i = 0 ; i < DRIVE_MOTO_NUM ; i++) 
	{	
		speed[i] += g_speed_encoder[ i ].speed ;
		pwm[i] += g_CarCtrl.moto_drive[ i ] ;
	}
	
	if ( index < 9 ) 
	{
		index++;
	}
	else
	{
		index = 0 ;
		speed[0] = speed[0] / 10 ;
		speed[1] = speed[1] / 10 ;
		pwm[0] = pwm[0] / 10 ;
		pwm[1] = pwm[1] / 10 ;
		sprintf( buf , "L:%d R:%d" , speed[0] , speed[1] );
		OLED_ShowAscii( 0,0, buf , 16 ,0 );
		
		sprintf( buf , "L:%d R:%d" , pwm[0] , pwm[1] );
		OLED_ShowAscii( 1,0, buf , 16 ,0 );
		
		speed[0] = 0 ;
		speed[1] = 0 ;		
		pwm[0] = 0 ;
		pwm[1] = 0 ;
	}
}

void CarCtrl_Process( void )
{
	if ( g_car_ctrl_state == CarCtrl_STOP ) return ;
	if ( g_car_ctrl_state == CarCtrl_START ) 
	{
		g_car_ctrl_state = CarCtrl_IDLE ;
		Speed_Calculate();
		CarCtrl_Speed_PID();
	//	CarCtrl_Show();
		CarCtrl_PlanSet();
		 // make it stop when operating once
	}
}
//妈的你们不写注释还包的一层又一层。。。。。。靠
//你这不是优秀的解耦。是优秀的刑具。。。。。。
