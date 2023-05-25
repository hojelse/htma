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

#define MIN_DEG -180
#define MAX_DEG 180
#define PROGRAM_BUFFER_SIZE 10
short program_array_fst[PROGRAM_BUFFER_SIZE] = {0};
short program_size_fst = 1;
short program_array_snd[PROGRAM_BUFFER_SIZE] = {0};
short program_size_snd = 1;
short program_index = -1;

#define STEP_PIN(arm) ((arm == Fst) ? 11 : ((arm == Snd) ? 9 : -1))
#define DIR_PIN(arm) ((arm == Fst) ? 10 : ((arm == Snd) ? 8 : -1))
#define STEPS_PER_DEG(arm) ((arm == Fst) ? (200 / 360.0) : ((arm == Snd) ? (200 / 360.0) : 0))
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
      execute_program_frame();
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
      program_array_fst[i] < MIN_DEG || 
      program_array_fst[i] > MAX_DEG || 
      program_array_snd[i] < MIN_DEG || 
      program_array_snd[i] > MAX_DEG
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
  int arm_link = command.substring(0,command.indexOf(' ')).toInt(); 
  int deg_move = command.substring(command.indexOf(' ')+1).toInt();
  
  switch(arm_link) {
    case 1: revolve(Fst, deg_move); break;
    case 2: revolve(Snd, deg_move); break;
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

void execute_program_frame() {
  Arm longest_mover;
  Arm shortest_mover;
  int longest_mover_steps;
  int shortest_mover_steps;
  {
    int fst_move_steps;
    int snd_move_steps;
    {
      int fst_move_deg = program_array_fst[program_index];
      int snd_move_deg = program_array_snd[program_index];
      if (fst_move_deg > 0) { digitalWrite(DIR_PIN(Fst), LOW); } else { digitalWrite(DIR_PIN(Fst), HIGH); fst_move_deg = abs(fst_move_deg); }
      if (snd_move_deg > 0) { digitalWrite(DIR_PIN(Snd), LOW); } else { digitalWrite(DIR_PIN(Snd), HIGH); snd_move_deg = abs(snd_move_deg); }
      fst_move_steps = fst_move_deg * STEPS_PER_DEG(Fst);
      snd_move_steps = snd_move_deg * STEPS_PER_DEG(Snd);
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
      longest_mover = Fst; longest_mover_steps = fst_move_steps;
      shortest_mover = Snd; shortest_mover_steps = snd_move_steps;
    }
    else {
      longest_mover = Snd; longest_mover_steps = snd_move_steps;
      shortest_mover = Fst; shortest_mover_steps = fst_move_steps;
    }
  }

  double coefficient = (shortest_mover_steps / (double)(longest_mover_steps - shortest_mover_steps));
  double counter = 0;
  int shortest_step_count = 0;
  digitalWrite(STEP_PIN(Fst), LOW);
  digitalWrite(STEP_PIN(Snd), LOW);
  while(longest_mover_steps--) {
    digitalWrite(STEP_PIN(longest_mover), HIGH);
    if (counter >= 1.0) {
      digitalWrite(STEP_PIN(shortest_mover), HIGH);
      shortest_step_count++;
      counter -= 1;
    }
    else counter += coefficient;
    delayMicroseconds(10);
    digitalWrite(STEP_PIN(longest_mover), LOW);
    digitalWrite(STEP_PIN(shortest_mover), LOW);
    delay(MOTOR_DELAY_MS);
  }
  while((shortest_step_count++) < shortest_mover_steps) {
    digitalWrite(STEP_PIN(shortest_mover), HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN(shortest_mover), LOW);
    delay(MOTOR_DELAY_MS);
  }
}

void revolve(Arm arm, int deg) {
  if (0 < deg) {
    digitalWrite(DIR_PIN(arm), LOW);
  } else if (deg < 0) {
    digitalWrite(DIR_PIN(arm), HIGH);
    deg = abs(deg);
  } else {
    return;
  }

  int steps = deg * STEPS_PER_DEG(arm);

  Serial.print("Rotating ");
  Serial.print(arm + 1);
  Serial.print(" ");
  Serial.print(deg);
  Serial.print(" degrees (");
  Serial.print(steps);
  Serial.println(" steps)");

  digitalWrite(STEP_PIN(arm), LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN(arm), HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN(arm), LOW);
    delay(MOTOR_DELAY_MS);
  }
}
