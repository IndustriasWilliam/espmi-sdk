# espmi-sdk

SDK base para a placa ESPMI HDMI com `ESP32-S3`, saida de video via `TFP410` e interface grafica em `LVGL`.

O projeto atual sobe uma interface grafica em `640x480`, inicializa a ponte HDMI, integra o display com o `LVGL` e opcionalmente recebe toque por `USB HID`.

## Visao geral

Este repositorio deixou de ser apenas o exemplo generico de RGB LCD da Espressif e passou a funcionar como base de firmware para a familia ESPMI HDMI.

Hoje ele entrega:

- inicializacao de video HDMI sobre barramento RGB de 16 bits
- deteccao e configuracao da ponte `TFP410` por `I2C`
- integracao com `LVGL 8.3.6`
- demo grafica pronta para validar display e renderizacao
- suporte a touch USB via `usb_host_hid`
- perfil de renderizacao ajustavel para baixa latencia ou animacao pesada

## Stack usada

- MCU: `ESP32-S3`
- Framework: `ESP-IDF >= 5.0`
- GUI: `LVGL ~8.3.6`
- Video: `TFP410 RGB-to-HDMI bridge`
- Entrada: `USB HID`

As dependencias principais estao declaradas em:

- [main/idf_component.yml](main/idf_component.yml)
- [components/HDMI/idf_component.yml](components/HDMI/idf_component.yml)

## Estrutura do projeto

- [main/main.c](main/main.c): aplicacao principal, setup do HDMI, touch USB e demo do LVGL
- [components/HDMI/include/HDMI.h](components/HDMI/include/HDMI.h): API publica da camada HDMI
- [components/HDMI/HDMI.c](components/HDMI/HDMI.c): inicializacao da ponte, display, buffers, touch e ciclo do LVGL
- [sdkconfig.defaults.esp32s3](sdkconfig.defaults.esp32s3): configuracoes base para o alvo `esp32s3`
- [managed_components](managed_components): dependencias baixadas pelo ESP-IDF na primeira compilacao

## Requisitos

Antes de gravar a placa, tenha em maos:

- ambiente `ESP-IDF 5.x` configurado
- cabo USB para programacao e alimentacao
- monitor ou TV com entrada HDMI
- cabo HDMI
- placa ESPMI HDMI baseada em `ESP32-S3`

Opcional:

- dispositivo de toque USB compativel

## Build rapido

Defina o alvo:

```bash
idf.py set-target esp32s3
```

Compile o projeto:

```bash
idf.py build
```

Grave e abra o monitor serial:

```bash
idf.py -p COM5 flash monitor
```

Troque `COM5` pela porta correta da sua maquina.

Na primeira compilacao, o `ESP-IDF` pode baixar dependencias para a pasta `managed_components`.

## O que o firmware faz hoje

No estado atual do projeto, o fluxo principal e este:

1. Carrega a configuracao padrao da camada HDMI.
2. Configura `I2C` da ponte `TFP410`.
3. Define os GPIOs do barramento RGB e dos sinais de sincronismo.
4. Sobe o video em `640x480`.
5. Habilita o toque USB, quando ativo.
6. Inicializa o `LVGL`.
7. Executa a demo `lv_demo_widgets()`.

Se a opcao `run_benchmark_demo` for ligada em [main/main.c](main/main.c), a aplicacao passa a usar `lv_demo_benchmark()`.

## Configuracoes importantes

As configuracoes mais uteis para o primeiro ajuste estao em [main/main.c](main/main.c):

- `hdmi_config.i2c.*`: pinos e clock do `I2C`
- `hdmi_config.panel_config.*`: pinos do barramento RGB e timings de video
- `hdmi_config.usb_touch.*`: calibracao e comportamento do touch USB
- `HDMI_ApplyRenderProfile(...)`: perfil de desempenho/renderizacao

Os valores padrao atuais incluem:

- `I2C SDA`: `GPIO_NUM_5`
- `I2C SCL`: `GPIO_NUM_4`
- `PCLK`: `GPIO_NUM_8`
- `VSYNC`: `GPIO_NUM_7`
- `HSYNC`: `GPIO_NUM_6`
- `DE`: `GPIO_NUM_15`
- resolucao: `640x480`
- `pclk_hz`: `24000000`

## Touch USB

O projeto ja contem a infraestrutura para receber relatorios HID e traduzir isso em eventos de toque para o `LVGL`.

Os ajustes mais comuns ficam nestes campos:

- `raw_max_x`
- `raw_max_y`
- `swap_xy`
- `invert_x`
- `invert_y`

Se o toque responder invertido ou fora de escala, normalmente o ajuste comeca por ai.

## Logs esperados

Quando tudo sobe corretamente, voce deve observar no monitor serial sinais de que:

- a pilha USB foi inicializada
- a ponte `TFP410` respondeu no `I2C`
- o display foi criado
- o `LVGL` entrou em execucao

Se a interface aparecer na tela, o pipeline principal esta validado.

## Troubleshooting

### Sem imagem no HDMI

Confira:

- se o monitor esta na entrada correta
- se o cabo HDMI esta funcionando
- se a placa esta energizada e gravada corretamente
- se a ponte `TFP410` foi detectada no log serial

### Touch sem resposta

Confira:

- se o dispositivo USB e realmente HID
- se o USB host foi iniciado com sucesso
- se `raw_max_x` e `raw_max_y` batem com o hardware usado
- se precisa inverter eixo ou trocar `x/y`

### Interface lenta

O desempenho depende principalmente de:

- resolucao
- perfil de renderizacao
- altura do draw buffer
- quantidade de atualizacoes simultaneas na tela

Para o primeiro bring-up, o ideal e buscar estabilidade antes de otimizar animacoes.

## Proximos passos

Uma evolucao natural deste projeto e:

1. manter a inicializacao HDMI como esta
2. trocar a demo do `LVGL` por uma UI propria
3. encapsular telas, widgets e eventos em modulos separados
4. calibrar o touch para o hardware final
5. documentar o hardware da placa junto com o firmware

## Referencias uteis

- [ESP-IDF Getting Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)
- [LVGL Documentation](https://docs.lvgl.io/)
