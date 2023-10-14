# TwoMotors
  Biblioteca desenvolvida para o uso com a plataforma Arduino com o objetivo de facilitar o uso de dois motores - que utilizem sensor encoder - com a biblioteca <a href="https://github.com/hugo-max-m-teixeira/DC_motor_controller">DC_motor_controlller</a>

## Configurando
### Instalação
  Primeiro, é necessário que você instale a bilbioteca no Arduino IDE. Caso ainda não saiba fazer isso, recomendamos que baixe o arquivo .zip e adicione pelo gerenciador de bibliotecas do Arduino.

### Criando o obbeto (instanciando)

```cpp

#include <DC_motor_controller.h> // Para o controle de cada motor, individualmente.
#include <TwoMotors.h> // Para controle de dois motores simultaneamente

// Agora instanciamos dois objetos da classe DC_motor_controller, que chamaremos de motorL (representando o motor esquerdo)
// e motorR (representado o motor direito)

DC_motor_controller motorR, motorL;

// Agora criaremos um ojeto da classe TwoMotors para controlarmos ambos os motores ao mesmo tempo,
// informando como parâmentro os objetos dos dois motores;
TwoMotors both(motorR, motorL);  // Nesse caso, o objeto é chamado "both" (ambos - em inglês)


```
