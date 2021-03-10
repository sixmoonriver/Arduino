
//原有代码逻辑
loop(){
	//接收数据分开是怕因为发送信号期间影响接收遥控信号
	if(接收数据){
		if(是否开关机信号)
			上一次状态=当前状态
			当前状态翻转
	接收下个数据
	}
	//发送开机代码需要持续的进行，而不是只在按键时进行一次
	if(当前状态开机>){
		if(上次状态是开机)
			发送开机代码
		else
			发送关机代码
	}
	else{
		if(上次状态是开机)
			发送关机代码
		else
			打印已关机
		
	}
}

unsigned long count=0;
unsigned long lasttime =0;
int interval=5000000; 

//修改逻辑，减小开机代码发送次数，以减少对收音机的干扰。
loop(){
	if(接收数据){
		if(是否开关机信号)
			if(当前状态开机)
				发送关机代码
			else 
				发送开机代码
				lasttime=millis()；
			状态翻转
			
	接收下个数据
	}
	//状态如果是开机，如果持续时间超过间隔，发送开机代码，重置lasttime；
	if(当前状态是开机){
		if((millis()-lasttime)>=interval)
			发送开机代码
			lasttime=millis();
	}

}
