/*
 Stepper Motor Control - revolution
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

#define MIN_DEG -180
#define MAX_DEG 180
#define PROGRAM_BUFFER_SIZE 10
short program_array_fst[PROGRAM_BUFFER_SIZE] = {0};
short program_size_fst = 1;
short program_array_snd[PROGRAM_BUFFER_SIZE] = {0};
short program_size_snd = 1;
short program_index = -1;

#define STEP_PIN 11
#define DIR_PIN 10
#define STEPS_PER_REV 200
#define STEPS_PER_DEG_FST (200 / 360.0)
#define STEPS_PER_DEG_SND (0 / 360.0)


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
      if (program_index >= program_size_fst) break;
      Serial.write("Executing frame [");
      Serial.print(program_array_fst[program_index]);
      Serial.write(",");
      Serial.print(program_array_snd[program_index]);
      Serial.write("]...\n");
      revolve(program_array_fst[program_index]);
      delay(1);
      program_index++;
    }
    default: break;
  }
}

void verify_program() {
  program_index = -1;
  if (program_size_fst != program_size_snd) {Serial.write("The current program arrays have differing lengths...\n"); return;}
  for(short i = 0; i < program_size_fst; i++) {
    if (
      program_array_fst[i] < MIN_DEG || 
      program_array_fst[i] > MAX_DEG || 
      program_array_snd[i] < MIN_DEG || 
      program_array_snd[i] > MAX_DEG
    ) {Serial.write("The current program breaks limits...\n"); return;}       
  }
  Serial.write("Program verified!\n");
  program_index = 0;
}

void execute_command(String command) {
  if (command.charAt(0) == ':') switch(command.charAt(1)) {
    case Run:
      if (program_index < 0) { Serial.write("The loaded program is not verified, so I cannot run it :(\n"); break;}
      program_index = 0;
    case Homing:
    case Program:
      Serial.write(("Switching mode\n"));
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
    case 'f': {
      Serial.write("Loading new fst array...\n");
      load_program_array(command.substring(command.indexOf(' ')+1), program_array_fst, &program_size_fst);
      program_index = -1;
      break;
    }
    case 's':  {
      Serial.write("Loading new snd array...\n");
      load_program_array(command.substring(command.indexOf(' ')+1), program_array_snd, &program_size_snd);
      program_index = -1;
      break;
    }
  }
}

String trim_start(String data) {
  int index = 0;
  while(index < data.length()-1) if (data.charAt(index) == ' ') {index++;} else {break;}
  return data.substring(index);
}

String trim_end(String data) {
  int index = data.length()-1;
  while(index > 0) if (data.charAt(index) == ' ') {index--;} else {break;}
  return data.substring(0, index);
}

void load_program_array(String data, short* array, short* array_size) {

  data = trim_start(trim_end(data));

  int i = 0;
  while(i < PROGRAM_BUFFER_SIZE) {
    if (data == "") break;

    if (data.indexOf(' ') == -1) {
      array[i] = (short)data.toInt();
      data = "";
    }
    else {
      array[i] = (short)data.substring(0, data.indexOf(' ')).toInt();
      data = trim_start(data.substring(data.indexOf(' ')));
    }
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

  int steps = deg * STEPS_PER_DEG_FST;

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
