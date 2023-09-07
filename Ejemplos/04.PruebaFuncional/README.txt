NOTAS:
-Este ejemplo muestra de forma explicita que es posible obtener datos de GNSS, un "fix" y ver datos de localizacion posteriormente.
-Prestar atencion especial a prj.conf: Cerca del final se encuentran las siguientes configuraciones:

CONFIG_MODEM_ANTENNA_GNSS_EXTERNAL=y
#CONFIG_MODEM_ANTENNA_AT_MAGPIO="AT%XMAGPIO=1,0,0,1,1,1574,1577"
#CONFIG_MODEM_ANTENNA_AT_COEX0="AT%XCOEX0=1,1,1565,1586"

estas aparecen cuando se escogen las opciones de mismo nombre en la interfaz grafica de KConfig y guardando bajo "save to file"

la primera linea permite utilizar antena externa, pero mantener las otras dos lineas descomentadas hace que dicha configuracion sea ignorada.
las configuraciones de antena se mantienen por defecto incluso si se cambia de estilo de antena. Se asume que esto es simplemente
un problema de configuración por quien sea programó el ejemplo. para que pueda utilizarse antena externa, entonces, deben comentarse
las lineas especificas de configuracion de antena y que así se utilicen las por defecto de la placa (tal que CONFIG_MODEM_ANTENNA_GNSS_EXTERNAL=y sobreescriba).

De manera experimental: Utilizar las tres lineas hace que el ejemplo consiga un fix en runtime tanto con antena externa conectada
como desconectada. Sin embargo, al descomentar las dos lineas inferiores hace que solo detecte satelites cuando la atena externa este conectada.
