/*
 Stepper Motor Control - revolution
 */

enum Mode {
  Manual = 'm',
  Program = 'p',
  Run = 'r'
};

enum Arm {
  Fst,
  Snd
};

char mode = Manual;

#define INPUT_BUFFER_SIZE 20
char input_buffer[INPUT_BUFFER_SIZE + 1];
char input_index = 0;

#define FREE_DEG_FST 90
#define FREE_DEG_SND 130
#define TOTAL_ARM_LENGTH 36
short fst_curr_angle = 0;
short snd_curr_angle = 0;
#define PROGRAM_BUFFER_SIZE 10
short sequence_fst[PROGRAM_BUFFER_SIZE] = {0};
short sequence_size_fst = 1;
short sequence_snd[PROGRAM_BUFFER_SIZE] = {0};
short sequence_size_snd = 1;
short program_index = -1;
bool paused = false;

#define STEPS_FST_MOTOR 3200.0
#define STEPS_SND_MOTOR 3200.0
#define RATIO_FST_MOTOR 2.857143
#define RATIO_SND_MOTOR 1.0
#define MOVE_DIR(dir, arm) (dir) ? (((arm == Fst) ? LOW : ((arm == Snd) ? LOW : HIGH))) : ((arm == Fst) ? HIGH : ((arm == Snd) ? HIGH : LOW))
#define STEP_PIN(arm) ((arm == Fst) ? 11 : ((arm == Snd) ? 9 : -1))
#define DIR_PIN(arm) ((arm == Fst) ? 10 : ((arm == Snd) ? 8 : -1))
#define STEPS_PER_DEG(arm) ((arm == Fst) ? ((STEPS_FST_MOTOR * RATIO_FST_MOTOR) / 360.0) : ((arm == Snd) ? ((STEPS_SND_MOTOR * RATIO_SND_MOTOR) / 360.0) : 0))
short motor_delay_ms = 15;

#define RAD_TO_DEG_FACTOR 180/3.1415926535

String trim_start(String data) {
  int index = 0;
  while(index < data.length()-1) if (data.charAt(index) == ' ') {index++;} else {break;}
  return data.substring(index);
}

String trim_end(String data) {
  int index = data.length()-1;
  while(index > 0) if (data.charAt(index) == ' ') {index--;} else {break;}
  return data.substring(0, index+1);
}

void setup() {
  pinMode(STEP_PIN(Fst), OUTPUT);
  pinMode(STEP_PIN(Snd), OUTPUT);
  pinMode(DIR_PIN(Fst), OUTPUT);
  pinMode(DIR_PIN(Snd), OUTPUT);

  Serial.begin(9600); // initialize the serial port
  verify_program();
}

void loop() {
  if (Serial.available()) execute_command(Serial.readString());
  switch(mode) {
    case Run: {
      if (paused || program_index >= sequence_size_fst) break;
      Serial.write("Executing frame [");
      Serial.print(sequence_fst[program_index]);
      Serial.write(",");
      Serial.print(sequence_snd[program_index]);
      Serial.write("]\n");
      execute_movement(sequence_fst[program_index], sequence_snd[program_index]);
      delay(1);
      program_index++;
    }
    default: break;
  }
}

bool verify_program() {
  program_index = -1;
  if (sequence_size_fst != sequence_size_snd) {Serial.write("Error: Different lengths\n"); return false;}
  for(short i = 0; i < sequence_size_fst; i++) {
    if (
      sequence_fst[i] < -FREE_DEG_FST || 
      sequence_fst[i] > FREE_DEG_FST || 
      sequence_snd[i] < -FREE_DEG_SND || 
      sequence_snd[i] > FREE_DEG_SND
    ) {Serial.write("Error: Broken limits\n"); return false;}       
  }
  Serial.write("Program verified!\n");
  program_index = 0;
  return true;
}

void execute_command(String command) {
  if (command.charAt(0) == ':') switch(command.charAt(1)) {
    case Run:
      if (program_index < 0) { Serial.write("Error: Non-verified program\n"); break;}
      paused = false;
      program_index = 0;
      Serial.write("Running\n");
    case Manual:
    case Program:
      Serial.write("Mode switched\n");
      mode = command.charAt(1);
      break;
    default:
      Serial.write("Error: Unknown mode\n");
  }
  else switch(mode) {
    case Manual: handle_Manual_command(command); break;
    case Program: handle_program_command(command); break;
    case Run: handle_run_command(command); break;
    default: break; 
  }
}

void handle_Manual_command(String command) {
  switch(command.charAt(0)) {
    case 'w': {
      Serial.write("f:"); Serial.println(fst_curr_angle);
      Serial.write("s:"); Serial.println(snd_curr_angle);
      return;
    }
    case 'z': {
      Serial.write("Zeroed\n");
      fst_curr_angle = 0;
      snd_curr_angle = 0;
      return;
    }
    case 'm': {
      int new_delay = command.substring(command.indexOf(' ')+1).toInt();
      if (new_delay < 6 || new_delay > 100) Serial.println("Error: Unacceptable delay");
      else {Serial.println("Updating motor delay"); motor_delay_ms = new_delay;}
      return;
    }
    case 'f': {
      int deg_move = command.substring(command.indexOf(' ')+1).toInt();
      execute_movement(fst_curr_angle+deg_move, snd_curr_angle);
      return;
    }
    case 's': {
      int deg_move = command.substring(command.indexOf(' ')+1).toInt();
      execute_movement(fst_curr_angle, snd_curr_angle+deg_move);
      return;
    }
    default: Serial.println("Error: Unknown command");
  }
}

void handle_program_command(String command) {
  switch(command.charAt(0)) {
    case 'v': { verify_program(); break; }
    case 'p': { print_program(); break;}
    case 'f': {
      load_program_array(command.substring(command.indexOf(' ')+1), sequence_fst, &sequence_size_fst);
      Serial.println("Loaded array!");
      program_index = -1;
      break;
    }
    case 's':  {
      load_program_array(command.substring(command.indexOf(' ')+1), sequence_snd, &sequence_size_snd);
      Serial.println("Loaded array!");
      program_index = -1;
      break;
    }
    case 'c': {
      int temp_sequence_fst[PROGRAM_BUFFER_SIZE];
      int temp_sequence_snd[PROGRAM_BUFFER_SIZE];
      command = trim_start(trim_end(command.substring(2)));
      int i = 0;
      while(i < PROGRAM_BUFFER_SIZE) {
        double x = 0;
        double z = 0;
        if (command.length() <= 0) break;
        if (command.indexOf(' ') == -1) {
          x = command.substring(0, command.indexOf(',')).toDouble();
          z = command.substring(command.indexOf(',')+1).toDouble();
          command = "";
        }
        else {
          String coordinate = command.substring(0,command.indexOf(' ')+1);
          command = trim_start(command.substring(command.indexOf(' ')+1));
          x = coordinate.substring(0, coordinate.indexOf(',')).toDouble();
          z = coordinate.substring(coordinate.indexOf(',')+1).toDouble();
        }
        if (z < 0) {
          Serial.println("Error: Negative z-axis is not allowed!");
          return;
        }

        Serial.print("Calculating inverse kinematics for coordinate: (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(z);
        Serial.print(")");
        Serial.println();

        double s_angle = 0;
        double f_angle = 0;
        bool reachable = coords_to_angles(x, z, &s_angle, &f_angle);
        if(!reachable) {
          Serial.println("Error: Unreachable coordinate!");
          return;
        }

        temp_sequence_fst[i] = (int)round(f_angle);
        temp_sequence_snd[i] = (int)round(s_angle);

        i++;
      }
      for(int t = 0; t < i; t++) {
        sequence_fst[t] = temp_sequence_fst[t];
        sequence_snd[t] = temp_sequence_snd[t];
      }
      Serial.println("Loaded arrays!");
      program_index = -1;
      sequence_size_fst = i;
      sequence_size_snd = i;

      print_program();
      break;
    }
    default: Serial.println("Error: Unknown command");
  }
}

void print_program() {
  
  Serial.println("Program:");

  Serial.print("fst: [");
  for (int i = 0; i < sequence_size_fst; i++) {
    Serial.print(sequence_fst[i]);
    Serial.print("\t");
  }
  Serial.println("]");

  Serial.print("snd: [");
  for (int i = 0; i < sequence_size_snd; i++) {
    Serial.print(sequence_snd[i]);
    Serial.print("\t");
  }
  Serial.println("]");
}

bool coords_to_angles(double x, double z, double* s_angle, double* f_angle) {
  double dist = sqrt(x*x + z*z);
  double blah = atan(z/x) * RAD_TO_DEG_FACTOR;
  double theta = 2 * asin((dist/2)/18) * RAD_TO_DEG_FACTOR;

  if (dist > TOTAL_ARM_LENGTH) return false;

  *s_angle = 180 - theta;
  *f_angle = 90 - (180 - theta)/2 - blah;
  return true;
}

void handle_run_command(String command) {
  switch(command.charAt(0)) {
    case 'p': {
      paused = true;
      Serial.write("Pausing\n");
      return;
    }
    case 'r': {
      paused = false;
      Serial.write("Resuming\n");
      return;
    }
    default: Serial.println("Error: Unknown command");
  }
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

void execute_movement(short fst_move_to, short snd_move_to) {
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
      delay(motor_delay_ms);
    } 
    return; 
  }
  if (!snd_move_steps) { 
    digitalWrite(STEP_PIN(Fst), LOW);
    while(fst_move_steps--) {
      digitalWrite(STEP_PIN(Fst), HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN(Fst), LOW);
      delay(motor_delay_ms);
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
      delay(motor_delay_ms);
    }
    return;
  }

  if (fst_move_steps > snd_move_steps) {
    fst_step_delay = motor_delay_ms;
    snd_step_delay = (int)(motor_delay_ms * (fst_move_steps/(double)snd_move_steps));
  }
  else {
    snd_step_delay = motor_delay_ms;
    fst_step_delay = (int)(motor_delay_ms * (snd_move_steps/(double)fst_move_steps));
  }

  digitalWrite(STEP_PIN(Fst), LOW);
  digitalWrite(STEP_PIN(Snd), LOW);
  unsigned long latest_fst = millis();
  unsigned long latest_snd = latest_fst;
  bool fst_moved = false;
  bool snd_moved = false;
  while(fst_move_steps || snd_move_steps) {
    if (fst_move_steps && (millis() - latest_fst) >= fst_step_delay) {
      fst_move_steps--;
      fst_moved = true;
      latest_fst = millis();
      digitalWrite(STEP_PIN(Fst), HIGH);
    }
    if (snd_move_steps && (millis() - latest_snd) >= snd_step_delay) {
      snd_move_steps--;
      snd_moved = true;
      latest_snd = millis();
      digitalWrite(STEP_PIN(Snd), HIGH);
    }
    delayMicroseconds(10);
    if (fst_moved) {digitalWrite(STEP_PIN(Fst), LOW); fst_moved = false;}
    if (snd_moved) {digitalWrite(STEP_PIN(Snd), LOW); snd_moved = false;}
  }
}
