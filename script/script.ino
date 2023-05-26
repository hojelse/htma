/*
 Stepper Motor Control - revolution
 */

enum Mode {
  Homing = 'h',
  Program = 'p',
  Run = 'r'
};

enum Arm {
  Fst,
  Snd
};

char mode = Homing;

#define INPUT_BUFFER_SIZE 20
char input_buffer[INPUT_BUFFER_SIZE + 1];
char input_index = 0;

#define FREE_DEG_FST 90
#define FREE_DEG_SND 130
short fst_curr_angle = 0;
short snd_curr_angle = 0;
#define PROGRAM_BUFFER_SIZE 10
short program_array_fst[PROGRAM_BUFFER_SIZE] = {0};
short program_size_fst = 1;
short program_array_snd[PROGRAM_BUFFER_SIZE] = {0};
short program_size_snd = 1;
short program_index = -1;

#define MOVE_DIR(dir, arm) (dir) ? (((arm == Fst) ? HIGH : ((arm == Snd) ? LOW : HIGH))) : ((arm == Fst) ? LOW : ((arm == Snd) ? HIGH : LOW))
#define STEP_PIN(arm) ((arm == Fst) ? 11 : ((arm == Snd) ? 9 : -1))
#define DIR_PIN(arm) ((arm == Fst) ? 10 : ((arm == Snd) ? 8 : -1))
#define STEPS_PER_DEG(arm) ((arm == Fst) ? (3200 / 360.0) : ((arm == Snd) ? (3200 / 360.0) : 0))
#define MOTOR_DELAY_MS 15


void setup() {
  pinMode(STEP_PIN(Fst), OUTPUT);
  pinMode(STEP_PIN(Snd), OUTPUT);
  pinMode(DIR_PIN(Fst), OUTPUT);
  pinMode(DIR_PIN(Snd), OUTPUT);

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
      Serial.write("]\n");
      execute_program_frame(program_array_fst[program_index], program_array_snd[program_index]);
      delay(1);
      program_index++;
    }
    default: break;
  }
}

void verify_program() {
  program_index = -1;
  if (program_size_fst != program_size_snd) {Serial.write("Error: Different lengths\n"); return;}
  for(short i = 0; i < program_size_fst; i++) {
    if (
      program_array_fst[i] < -FREE_DEG_FST || 
      program_array_fst[i] > FREE_DEG_FST || 
      program_array_snd[i] < -FREE_DEG_SND || 
      program_array_snd[i] > FREE_DEG_SND
    ) {Serial.write("Error: Broken limits\n"); return;}       
  }
  Serial.write("Program verified!\n");
  program_index = 0;
}

void execute_command(String command) {
  if (command.charAt(0) == ':') switch(command.charAt(1)) {
    case Run:
      if (program_index < 0) { Serial.write("Error: Non-verified program\n"); break;}
      program_index = 0;
    case Homing:
    case Program:
      Serial.write("Mode switched\n");
      mode = command.charAt(1);
      break;
    default:
      Serial.write("Error: Unknown mode\n");
  }
  else switch(mode) {
    case Homing: handle_homing_command(command); break;
    case Program: handle_program_command(command); break;
    default: break; 
  }
}

void handle_homing_command(String command) {
  if (command.charAt(0) == 'z') {
    Serial.write("Zeroed\n");
    fst_curr_angle = 0;
    snd_curr_angle = 0;
    return;
  }
  if (command.charAt(0) == 'w') {
    Serial.print("f:");
    Serial.println(fst_curr_angle);
    Serial.print("s:");
    Serial.println(snd_curr_angle);
    return;
  }
  char arm_link = command.charAt(0); 
  int deg_move = command.substring(command.indexOf(' ')+1).toInt();
  
  switch(arm_link) {
    case 'f': execute_program_frame(fst_curr_angle+deg_move, snd_curr_angle); break;
    case 's': execute_program_frame(fst_curr_angle, snd_curr_angle+deg_move); break;
    default: Serial.write("Error: No such arm\n");
  }
}

void handle_program_command(String command) {
  switch(command.charAt(0)) {
    case 'v': { verify_program(); break; }
    case 'f': {
      load_program_array(command.substring(command.indexOf(' ')+1), program_array_fst, &program_size_fst);
      Serial.write("Loaded array!\n");
      program_index = -1;
      break;
    }
    case 's':  {
      load_program_array(command.substring(command.indexOf(' ')+1), program_array_snd, &program_size_snd);
      Serial.write("Loaded array!\n");
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

void execute_program_frame(short fst_move_to, short snd_move_to) {
  int fst_step_delay;
  int snd_step_delay;
  int fst_move_steps;
  int snd_move_steps;
  {
    int fst_move_deg = fst_move_to - fst_curr_angle;
    int snd_move_deg = fst_move_deg + (snd_move_to - snd_curr_angle);
    if (fst_move_deg > 0) { digitalWrite(DIR_PIN(Fst), MOVE_DIR(true, Fst)); } else { digitalWrite(DIR_PIN(Fst), MOVE_DIR(false, Fst)); fst_move_deg = abs(fst_move_deg); }
    if (snd_move_deg > 0) { digitalWrite(DIR_PIN(Snd), MOVE_DIR(true, Snd)); } else { digitalWrite(DIR_PIN(Snd), MOVE_DIR(false, Snd)); snd_move_deg = abs(snd_move_deg); }
    fst_move_steps = fst_move_deg * STEPS_PER_DEG(Fst);
    snd_move_steps = snd_move_deg * STEPS_PER_DEG(Snd);
    fst_curr_angle = fst_move_to;
    snd_curr_angle = snd_move_to;
  }

  // Handles simple cases
  if (!fst_move_steps) { 
    digitalWrite(STEP_PIN(Snd), LOW);
    while(snd_move_steps--) {
      digitalWrite(STEP_PIN(Snd), HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN(Snd), LOW);
      delay(MOTOR_DELAY_MS);
    } 
    return; 
  }
  if (!snd_move_steps) { 
    digitalWrite(STEP_PIN(Fst), LOW);
    while(fst_move_steps--) {
      digitalWrite(STEP_PIN(Fst), HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN(Fst), LOW);
      delay(MOTOR_DELAY_MS);
    } 
    return; 
  }
  if (fst_move_steps == snd_move_steps) {
    digitalWrite(STEP_PIN(Fst), LOW);
    digitalWrite(STEP_PIN(Snd), LOW);
    while(fst_move_steps--) {
      digitalWrite(STEP_PIN(Fst), HIGH);
      digitalWrite(STEP_PIN(Snd), HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN(Fst), LOW);
      digitalWrite(STEP_PIN(Snd), LOW);
      delay(MOTOR_DELAY_MS);
    }
    return;
  }

  if (fst_move_steps > snd_move_steps) {
    fst_step_delay = MOTOR_DELAY_MS;
    snd_step_delay = (int)(MOTOR_DELAY_MS * (fst_move_steps/(double)snd_move_steps));
  }
  else {
    snd_step_delay = MOTOR_DELAY_MS;
    fst_step_delay = (int)(MOTOR_DELAY_MS * (snd_move_steps/(double)fst_move_steps));
  }

  digitalWrite(STEP_PIN(Fst), LOW);
  digitalWrite(STEP_PIN(Snd), LOW);
  int millis_since_fst = 0;
  int millis_since_snd = 0;
  int prev_millis = millis();
  while(fst_move_steps || snd_move_steps) {
    if (fst_move_steps && millis_since_fst >= fst_step_delay) {
      fst_move_steps--;
      digitalWrite(STEP_PIN(Fst), HIGH);
      millis_since_fst = 0;
    }
    if (snd_move_steps && millis_since_snd >= snd_step_delay) {
      snd_move_steps--;
      digitalWrite(STEP_PIN(Snd), HIGH);
      millis_since_snd = 0;
    }
    delayMicroseconds(10);
    digitalWrite(STEP_PIN(Fst), LOW);
    digitalWrite(STEP_PIN(Snd), LOW);
    int temp = millis();
    millis_since_fst += temp - prev_millis;
    millis_since_snd += temp - prev_millis;
    prev_millis = temp;
  }

}
