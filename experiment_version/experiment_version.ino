// 2 библиотеки для работы с датчиком цвета
#include <MD_TCS230.h>
#include <FreqCount.h>
// библиотека для работы с сервоприводами
#include <Servo.h>
#include <stdint.h>

//Задаем кучу переменных
Servo firstServo;   // Первый сервопривод
Servo secondServo;  // Второй сервопривод
Servo thirdServo;   // Третий сервопривод

//пины для датчика цвета
#define  S2_OUT  33 // Пин для S2
#define  S3_OUT  31 // Пин для S1
#define  OE_OUT  47 // Пин для OUT

// Переменные для записи цвета
byte red = 0;
byte green = 0;
byte blue = 0;

// Задаем пины для мотора
int speed_control = 5;  // пин контроля скорости
int in1 = 4;  // первый вход
int in2 = 3;  // второй вход
int enA = 6;  // пин датчика тиков
int enB = 7;  // второй пин датчика тиков
int previous_a = LOW;

long ticks = 0; //переменная для отслеживания количества пройденных тиков
long previous_time; // переменная для отслеживания времени
int target_ticks = 0; // переменная для установки количества тиков, которого мы хотим достичь

int error = 0;  // отслеживание ошибки (на сколько тиков промахнулись)
int current_state = 0;  // текущее состояние конечного автомата для мотора
int current_ticks = 0;  // переменная, в которой мы наращиваем количество тиков
int correction = 0; // поправка на ошибку (пока непонятно, как эту поправку вычислять)

// переменные регуляции
double linear_coefficient = 0.5;  // линейный коэффициент
double integral_coefficient = 0; // интегральный коэффициент
double diff_coefficient = 2;  // дифференциальный коэффициент
long regulation_period = 50;  // период регуляции (количество мс, после которого снова происходит регулировка мотора)
double errors[3] = {0, 0, 0};
double integral_value = 0;
double prev_action = 0;

// переменные для мотора (ныне не используются, так как поворот теперь идет в тиках, а не в мм)
double wheel_diameter_mm = 80;  // диаметр круга, который вращает мотор (для нас это диаметр окружности, на которой лежат отверстия в шлюзе)
double math_pi = 3.14159265359; // священное число Пи
double wheel_length_mm = wheel_diameter_mm * math_pi; // длина окружности в мм
double ticks_per_revolution = 1100; // количество тиков на оборот (взято, по сути, приблизительно, поскольку тесты дают число то чуть больше, то чуть меньше)

void setup() {
  // Выставляем скорость обмена данными
  Serial.begin(115200);
  // Устанавливаем для мотора режим работы "выход"
  pinMode(speed_control, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, INPUT);
  pinMode(enB, INPUT);
  analogWrite(speed_control, 0);

  // Задаем пины сервоприводам
  firstServo.attach(11);
  secondServo.attach(12);
  thirdServo.attach(13);

  // Устанавливаем для датчика цвета режим работы "выход"
  pinMode(S2_OUT, OUTPUT);
  pinMode(S3_OUT, OUTPUT);
  pinMode(OE_OUT, INPUT);
}

// перевести миллиметры в тики (не используется)
int mm_to_ticks(double mm)
{
  double cm = mm / 10;
  double number_of_revolutions = mm / wheel_length_mm;
  return (int)(number_of_revolutions * ticks_per_revolution);
}

// вращение мотора вперед
void clockwise() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

// смена направления вращения мотора
void counterclockwise()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// остановка мотора
void stop() 
{
  analogWrite(speed_control, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

// запуск вращения мотора с установленной скоростью
void set_velocity(int velocity)
{
  velocity = constrain(velocity, -100, 100);
  if (velocity < 0) 
  {
    counterclockwise();
    analogWrite(speed_control, -1 * velocity);
  } else 
  {
    clockwise();
    analogWrite(speed_control, velocity);
  }
}

// подсчитать количество тиков
void count_ticks() 
{
  int current_a = digitalRead(enA);
  if (previous_a == LOW && current_a == HIGH) 
  {
    int current_b = digitalRead(enB);
    if (current_b == LOW)
    {
      ticks--;
    }
    else 
    {
      ticks++;
    }
  }
  previous_a = current_a;
}

// установить отметку тиков, которую хотим достичь
void set_target(int t_ticks)
{
  target_ticks = t_ticks;
  errors[0] = 0;
  errors[1] = 0;
  errors[2] = 0;
  integral_value = 0;
}

// сместить ошибки
void shift_errors(double new_error) {
  errors[2] = errors[1];
  errors[1] = errors[0];
  errors[0] = new_error;
}

// PID-регулятор (это очень сильное колдунство!)
double pid(double new_error) {
  shift_errors(new_error);
  integral_value += (errors[1] + errors[0]) / (2 * regulation_period);
  double action = linear_coefficient * errors[0] +
    integral_coefficient * integral_value +
    diff_coefficient * (3 * errors[0] - 4 * errors[1] + errors[2]) / (2 * regulation_period);
  if(action > 0){
    action += 65;
    }
  else{
    action -= 65;
    }
  return action;
}

// отрегулировать мотор по данным об ошибке
void regulate()
{
    double unconstrained_action = pid(error);
    double diff = unconstrained_action - prev_action;
    diff = constrain(diff, -100, 100);  // Prevent motor speed from changing too quickly
    double constrained_action = constrain(prev_action + diff, -100, 100);
    set_velocity(constrained_action);
    prev_action = constrained_action;
}

// отладочные данные
void print_debug_output() {
  Serial.print("Target ticks: "); Serial.println(target_ticks);
  Serial.print("Current ticks: "); Serial.println(ticks);
  Serial.print("Action: "); Serial.println(prev_action);
  Serial.println();
}

// Функция для смены позиции переборок (угол поворота сервоприводов)
void changePosition(int first_angle, int second_angle, int third_angle) {
  firstServo.write(first_angle + 77);
  secondServo.write(second_angle + 70);
  thirdServo.write(third_angle + 87);
}

// главный цикл
void loop() 
{
  //считаем тики
  count_ticks();

  // если прошло больше мс, чем период регуляции...
  if (millis() - previous_time > regulation_period) 
  {
    // считаем, насколько промахнулись
    error = target_ticks - ticks;
    // если ошибка по модулю больше единицы 
    //(нужно, чтобы мотор не дергался постоянно и вообще переходил дальше, чем в регуляцию), регулируем
    if(abs(error) > 1){
      regulate();
    }
    // если ошибка в допустимых пределах
    else{
      // устанавливаем скорость в ноль
      set_velocity(0);

      //Здесь задаются действия для конечного автомата состояний
      //Состояние 0: шлюз совмещен с отверстием в ведре и принимает в себя конфету, переборки выставляются в нейтральное положение,
      //подается команда на движение к датчику, автомат переходит в Состояние 1
      //Состояние 1: шлюз совмещен с датчиком цвета и считывает цвет, переключает переборки в нужное положение,
      //подается команда на движение к точке сброса, автомат переходит в Состояние 2
      //Состояние 2: шлюз совмещен с точкой сброса, конфета уходит вниз,
      //подается команда на движение к отверстию в ведре, автомат переходит в Состояние 0, цикл замкнулся.
      switch(current_state)
      {
        case 0:
          current_ticks += 100;
          set_target(current_ticks);
          current_state = 1;
          Serial.println(0);
          break;
        case 1:
          //код выбора цвета
          //код выставления переборок в нужное положение
          current_ticks += 485;
          set_target(current_ticks);
          current_state = 2;
          Serial.println(1);
          break;
        case 2:
          changePosition(0, 0, 0); //устанавливаем переборки в нейтральное положение
          current_ticks += 500;
          set_target(current_ticks);
          //correction += 3;
          //current_ticks -= correction;
          current_state = 0;
          Serial.println(2);
          break;
        }
      }
    previous_time = millis(); 
    print_debug_output();
  }
}

