
void startup_delay(void){
	for (volatile unsigned long i = 0; i < 50000; i++) { ; }
}
