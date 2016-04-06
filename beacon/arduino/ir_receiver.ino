#define FREQ   38000
#define BEACON 4
#define D_SIZE 8

#define WAIT_NEXT() delay(1000/(BEACON * D_SIZE) )
#define WAIT_HALF() delay(1000/(2 * BEACON * D_SIZE) )

int i = 0;
const uint8_t target = 0x57;
uint8_t prev;
uint8_t bit_rd1;
uint8_t bit_rd2;

void setup() {
  prev = 0;
  Serial.begin(115200);
}

int getIrSignal() {
  int i;
  uint8_t ir_signal;
  uint8_t res;
  
  // Nyquist
  i = 0;
  ir_signal = 0;
  while(2*D_SIZE - 1 > i++) {
	  if(ir_signal == target) { // Success
      prev = 1;
		  return 1;
 		}

		bit_rd1 = !digitalRead(7);
		WAIT_HALF();
		bit_rd2 = !digitalRead(7);
		ir_signal = (ir_signal << 1) | ( (bit_rd1 == bit_rd2) ? bit_rd2 : bit_rd1);
		WAIT_HALF();
  }
  
  // Fail (this time)
  res = prev | 0;
  prev = 0;
  return res;
}

void loop() {
	if(Serial.read() > 0) {
//		Serial.write(getIrSignal() );
		Serial.print(getIrSignal() );
	}
}
