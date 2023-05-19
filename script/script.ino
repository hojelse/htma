/*
 Stepper Motor Control -  revolution
 */

enum Mode {
  Homing = 'h',
  Program = 'p',
  Run = 'r'
};

char mode = Homing;

#define INPUT_BUFFER_SIZE 20
char input_buffer[INPUT_BUFFER_SIZE + 1];
char input_index = 0;

#define MIN_X -100
#define MAX_X 100
#define MIN_Y -100
#define MAX_Y 100
#define PROGRAM_BUFFER_SIZE 10
short program_array_x[PROGRAM_BUFFER_SIZE] = {0};
short program_size_x = 1;
short program_array_y[PROGRAM_BUFFER_SIZE] = {0};
short program_size_y = 1;
short program_index = 0;
char program_verified = 0;

#define STEP_PIN 11
#define DIR_PIN 10
#define STEPS_PER_REV 200


void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.begin(9600); // initialize the serial port:
  verify_program();
}

void loop() {
  if (Serial.available()) execute_command(Serial.readString());
  switch(mode) {
    case Run: {
      Serial.write("Moveing to [");
      Serial.write(program_array_x[program_index]);
      Serial.write(",");
      Serial.write(program_array_y[program_index]);
      Serial.write("]...\n");
      program_index++;
      delay(1000); // Actually move the arm here :)
      program_index = (program_index + 1) % PROGRAM_BUFFER_SIZE;
    }
    default: break;
  }
}

void verify_program() {
  program_verified = 0;
  if (program_size_x != program_size_y) {Serial.write("The current program arrays have differing lengths...\n"); return;}
  for(short i = 0; i < program_size_x; i++) {
    if (program_array_x[i] < MIN_X || program_array_x[i] > MAX_X || program_array_y[i] < MIN_Y || program_array_y[i] > MAX_Y) {Serial.write("The current program breaks point limits...\n"); return;}       
  }
  program_verified = 1;
}

void execute_command(String command) {
  if (command.charAt(0) == ':') switch(command.charAt(1)) {
    case Run:
      if (!program_verified) { Serial.write("The loaded program is not verified, so I cannot run it :(\n"); break;}
    case Homing:
    case Program:
      mode = command.charAt(1);
      break;
    default:
      Serial.write("I don't know that mode...\n");
  }
  else switch(mode) {
    case Homing: handle_homing_command(command); break;
    case Program: handle_program_command(command); break;
    default: break; 
  }
}

void handle_homing_command(String command) {
  int arm_link = command.substring(0,command.indexOf(' ')).toInt(); 
  int deg_move = command.substring(command.indexOf(' ')+1).toInt();
  
  if (arm_link > 2 || arm_link < 0) { Serial.write("Sorry, I only have 2 arms :(\n"); return;}      

  revolve(deg_move); // Should actually move the specified arm
}

void handle_program_command(String command) {
  switch(command.charAt(0)) {
    case 'v': { verify_program(); break; }
    case 'x': {
      load_program_array(command.substring(command.indexOf(' ')), program_array_x, &program_size_x);
      program_verified = 0;
      break;
    }
    case 'y':  {
      load_program_array(command.substring(command.indexOf(' ')), program_array_y, &program_size_y);
      program_verified = 0;
      break;
    }
  }
}

void load_program_array(String data, short* array, short* array_size) {
  int i = 0;
  while(i < PROGRAM_BUFFER_SIZE) {
    data.trim();
    if (data = "") break;
    array[i] = data.substring(0, data.indexOf(' ')).toInt();
    i++;
  }
  *array_size = i;  
}

void revolve(int deg) {
  if (0 < deg) {
    digitalWrite(DIR_PIN, LOW);
  } else if (deg < 0) {
    digitalWrite(DIR_PIN, HIGH);
    deg = abs(deg);
  } else {
    return;
  }

  double step_per_deg = STEPS_PER_REV / 360.0;

  int steps = deg * step_per_deg;

  Serial.print("Rotating ");
  Serial.print(deg);
  Serial.print(" degrees (");
  Serial.print(steps);
  Serial.println(" steps) ...");

  digitalWrite(STEP_PIN, LOW);

  int delayms = 15;

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delay(delayms);
  }
}
