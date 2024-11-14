Import("env")
env.Execute("openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c 'init' -c 'halt' -c 'stm32f1x unlock 0' -c 'reset halt' -c 'exit'")
# Mensaje de confirmación
print("Memoria Flash desbloqueada.")