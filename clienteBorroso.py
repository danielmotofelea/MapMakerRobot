import paho.mqtt.client as mqtt
import numpy as np
import sys
import time
import math
from matplotlib import pyplot as plt
import skfuzzy as fuzzy
from skfuzzy import control
from sympy import *

t = Symbol('t')

ipBroker = "192.168.0.0" # DIRECCION IP DEL BROKER

contador = 0

desviacionActual = 0.0
desviacionAnterior = 0.0
error = 0.0

tempMedia = 0
humMedia = 0 

valoresMapa = []

velocidad = 0

# [0] = u_izq || [1] = u_cen || [2] = u_der
valoresUltra = [0.0,0.0, 0.0]

# [0] = pos_x || [1] = pos_y || [2] = pos_z
valoresPosicion = [0,0,0]

# [0] = temperatura || [1] = humedad
valoresDHT = [0,0]
valoresDHTAnt = [0,0]

# Vectores donde se almacenaran la posicion que tienen los obstaculos respecto de la posicion actual del robot
# [0] = coordenada x || [1] = coordenada y
posObsIzqAct = [0.0, 0.0]
posObsCenAct = [0.0, 0.0]
posObsDerAct = [0.0, 0.0]

posAnt = [50.0,50.0] # [0] = pos x || [1] = pos y POSICIONES INICIALES PARA LA INTEGRAL
posAct = [50.0,50.0] # [0] = pos x || [1] = pos y POSICIONES INICIALES PARA LA INTEGRAL

tamaño = (101,101) # Tamaño de la matriz del mapa

mapa = np.zeros(tamaño) # Creacion de la matriz que realiza la funcion de mapa

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def on_message(client, userdata, message):
	
	global contador, velocidad
	
	print(message.topic)
	if (message.topic == 'robot/mapa'):
		mostrarMapa()
	
	if (message.topic == 'robot/distancia/u_izq'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresUltra[0] = int(str(message.payload.decode("utf-8")))
		contador = contador + 1
	if (message.topic == 'robot/distancia/u_cen'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresUltra[1] = int(str(message.payload.decode("utf-8")))
		contador = contador + 1
	if (message.topic == 'robot/distancia/u_der'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresUltra[2] = int(str(message.payload.decode("utf-8")))
		contador = contador + 1
		
	if (message.topic == 'robot/posicion/x'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresPosicion[0] = float(str(message.payload.decode("utf-8")))
		contador = contador + 1
	if (message.topic == 'robot/posicion/y'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresPosicion[1] = float(str(message.payload.decode("utf-8")))
		contador = contador + 1
		
	if (message.topic == 'robot/dht/temp'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresDHT[0] = int(str(message.payload.decode("utf-8")))
		contador = contador + 1
	if (message.topic == 'robot/dht/hum'):
		print("Mensaje Recibido " ,str(message.payload.decode("utf-8")))
		valoresDHT[1] = int(str(message.payload.decode("utf-8")))
		contador = contador + 1
	
	if (contador == 7):
		
		print("Se han recibido todos los datos")
		
		contador = 0 # Se pone a 0 para poder facilitar la siguiente interaccion
		calculoPosicionObstaculos() # Se realiza antes dado que despues se modificaran los valores de los ultrasonidos
		calculoPosicionRobot()
		if (valoresUltra[0] < 100 or valoresUltra[1] < 100 or valoresUltra[2] < 100):
			velocidad = ejecucionBorroso()
		else:
			velocidad = 0

		enviarVelocidad(velocidad)
		calculoTemperaturaHuedad()
		realizarMapeo()

def calculoPosicionObstaculos():
	global valoresUltra, posAct, posAnt, posObsCenAct, posObsDerAct, posObsIzqAct, posObsIzqAnt,  posObsDerAnt, posObsCenAnt
	
	# El vector contiene los angulos de orientacion de los sensores
	vectores = [-30,+0, +30]
	
	# Se calcula la posicion final del objeto en el espacio respecto de la posicion inicial del robot
	
	posObsIzqAct[0] = (valoresUltra[0]/10)*math.cos(vectores[0]) + posAct[0]
	posObsIzqAct[1] = (valoresUltra[0]/10)*math.sin(vectores[0]) + posAct[1]
	
	posObsCenAct[0] = (valoresUltra[1]/10)*math.cos(vectores[1]) + posAct[0]
	posObsCenAct[1] = (valoresUltra[1]/10)*math.sin(vectores[1]) + posAct[1]
	
	posObsDerAct[0] = (valoresUltra[2]/10)*math.cos(vectores[2]) + posAct[0]
	posObsDerAct[1] = (valoresUltra[2]/10)*math.sin(vectores[2]) + posAct[1]
	
	print ("Las posiciones de los objetos (IZQ-CEN-DER) son:  ")
	
	print (posObsIzqAct)
	print (posObsCenAct)
	print (posObsDerAct)

def calculoPosicionRobot():

	global valoresPosicion, posAnt, posAct
	
	vectorVelocidad = [0.0,0.0]
	
	print("La posicion del robot es: ")
	for i in range(2):
		valoresPosicion[i] =  valoresPosicion[i] * 9.8 					# Se modifica de g a m/s^2 para hallar el vector aceleracion
		vectorVelocidad[i] = integrate(valoresPosicion[i]*t,t)			# Se integra para obtener el vector velocidad
		posAct[i] = integrate(vectorVelocidad[i],(t,0,0.5)) + posAnt[i]
		
	print (posAct)
	
	# Se integra para obtener el vector posicion, 0.5 porque es el tiempo que se aplica la velocidad del borroso
	
def calculoTemperaturaHuedad():

	global valoresDHT, valoresDHTAnt, tempMedia, humMedia
	
	tempMedia = (valoresDHT[0] + valoresDHTAnt[0])/2
	humMedia = (valoresDHT[1] + valoresDHTAnt[1])/2
	
	print("La temperatura y huemdad media son: ")
	print(tempMedia)
	print(humMedia)
	
	for i in range(2):
		valoresDHTAnt[i] = valoresDHT[i]
	
def realizarMapeo():

	global mapa, posAct, posAnt, posObsCenAct, posObsDerAct, posObsIzqAct
	
	posActAux = [0,0] 
	posObsCenActAux = [0,0]
	posObsDerActAux = [0,0]
	posObsIzqActAux = [0,0]
	
	for i in range(2):
		
		posActAux[i] = int (posAct[i])
		
		if (posObsCenAct[i]>=0):
			posObsCenActAux[i] = posObsCenAct[i] + 50
		else:
			posObsCenActAux[i] = -1*posObsCenAct[i]	
		posObsCenActAux[i] = int (posObsCenAct[i])
	
		if (posObsDerAct[i]>=0):
			posObsDerActAux[i] = posObsDerAct[i] + 50
		else:
			posObsDerActAux[i] = -1*posObsDerAct[i]	
		posObsDerActAux[i] = int (posObsDerAct[i])
		
		if (posObsIzqAct[i]>=0):
			posObsIzqActAux[i] = posObsIzqAct[i] + 50
		else:
			posObsIzqActAux[i] = -1*posObsIzqAct[i]
		posObsIzqActAux[i] = int (posObsIzqAct[i])
		
	print("Las posiciones despues del ajuste son: (ROBOT-IZQ-CEN-DER)")
	
	print(posAct)
	print(posObsIzqActAux)
	print(posObsCenActAux)
	print(posObsDerActAux)
	
	# En el mapa se tartan valores enteros tratados en valores del S.I.

	for i in range(len(mapa)):
		for j in range(len(mapa[i])):
				#if (posActAux[0] == i and posActAux[1] == j and mapa[i][j] == 0):
				#	mapa[i][j] = 8
				if (posObsIzqActAux[0] == i and posObsIzqActAux[1] == j and mapa[i][j] == 0):
					mapa[i][j] = 8
				if (posObsCenActAux[0] == i and posObsCenActAux[1] == j and mapa[i][j] == 0):
					mapa[i][j] = 8
				if (posObsDerActAux[0] == i and posObsDerActAux[1] == j and mapa[i][j] == 0):
					mapa[i][j] = 8		
	
	
	# Todos los valores actuales pasan a ser los anteriores en la siguiente interaccion
	for i in range(2):
		posAnt[i] = posAct[i]					# Se igualan las posiciones para las futuras iterracciones		

def mostrarMapa():
	
	global mapa
	
	#visualizamos la matriz

	plt.matshow(mapa, vmin=0,vmax=15)

	# (es necesario indicar vmin y vmax para que pyplot sepa que el minimo es 0 y el maximo 5)
	# (solo imagenes escala de grises)

	plt.show()
		

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def calculoRepulsion(valoresUltra, error, desviacionAnterior, desviacionActual):
	
	xFinal = 0
	yFinal = 0
	
	# El vector contiene los angulos de orientacion de los sensores
	vectores = [-30,+0, +30]
	#valoresUltra[1] = valoresUltra[1]-7 # Debido al montaje Hardware
	
	for i in range(3):
		
		#valoresUltra[i] = valoresUltra[i]/100.0 # DIVIDIR ENTRE 100 PARA OBTENER EL VALOR EN METROS
		
		if valoresUltra[i] > 100:
			valoresUltra[i] = 100 # SI EL VALOR DEL SENSOR ES MAYOR A UN METRO NO SE TIENE EN CUENTA Y SE IGUALA A 1 METRO
		#print(valoresUltra[i])

		vectorX = math.cos(vectores[i]*math.pi/180)*valoresUltra[i] # se calcula el vector unitario en X
		vectorY = math.sin(vectores[i]*math.pi/180)*valoresUltra[i] # se calcula el vector unitario en Y
		
		xFinal = xFinal + vectorX
		yFinal = yFinal + vectorY
		
		yFinal = round(yFinal, 4)
		xFinal = round(xFinal, 4)
	
	anguloBorroso = math.atan2(yFinal, xFinal)
	
	desviacionAnterior = desviacionActual
	desviacionActual = anguloBorroso
	error = desviacionActual - (desviacionAnterior)
	
	return error, desviacionAnterior, desviacionActual
	
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def borroso(error, desviacionActual):
	
		# SE INICIALIZAN LAS GRAFICAS 
	errorDesviacion = np.arange(-2*math.pi, 2*math.pi+0.1, 0.1)
	anguloActual = np.arange(-math.pi, math.pi+0.1, 0.1)
	velocidadSalida = np.arange(-1, 1.1, 0.1)
	
	entActIzq = fuzzy.trimf(anguloActual, [-math.pi, -math.pi, 0])
	entActCen = fuzzy.trimf(anguloActual, [-math.pi, 0, math.pi])
	entActDer = fuzzy.trimf(anguloActual, [0, math.pi, math.pi])
	
	entErrorIzq = fuzzy.trimf(errorDesviacion, [-2*math.pi, -2*math.pi, 0])
	entErrorCen = fuzzy.trimf(errorDesviacion, [-2*math.pi, 0, 2*math.pi])
	entErrorDer = fuzzy.trimf(errorDesviacion, [0, 2*math.pi, 2*math.pi])
	
	velocidadIzq = fuzzy.trimf(velocidadSalida, [-1, -1, -0.5])
	velocidadCen_Izq = fuzzy.trimf(velocidadSalida,[-1, -0.5, 0])
	velocidadCen = fuzzy.trimf(velocidadSalida, [-0.5, 0, 0.5])
	velocidadCen_Der = fuzzy.trimf(velocidadSalida,[0, 0.5, 1])
	velocidadDer = fuzzy.trimf(velocidadSalida, [0.5, 1, 1])
	
	# PINTAMOS LAS GRAFICAS PARA UNA MEJOR VISUALIZACION
	# figuras, (graficaAnguloActual, graficaErrorDesviacion, graficavelocidadSalida) = plt.subplots(nrows = 2, figsize = (8, 9))
	
	# graficaAnguloActual.plot(anguloActual,entActIzq, 'b', linewidth=1.5, label = 'Izq')
	# graficaAnguloActual.plot(anguloActual,entActCen,'r', linewidth=1.5, label = 'Cen')
	# graficaAnguloActual.plot(anguloActual,entActDer,'g', linewidth=1.5, label = 'Der')
	# graficaAnguloActual.set_title('Angulo FRR Actual')
	# graficaAnguloActual.legend()
	
	# graficaErrorDesviacion.plot(errorDesviacion,entErrorIzq, 'b', linewidth=1.5, label = 'Izq')
	# graficaErrorDesviacion.plot(errorDesviacion,entErrorCen,'r', linewidth=1.5, label = 'Cen')
	# graficaErrorDesviacion.plot(errorDesviacion,entErrorDer,'g', linewidth=1.5, label = 'Der')
	# graficaErrorDesviacion.set_title('Error Desviacion')
	# graficaErrorDesviacion.legend()

	# graficavelocidadSalida.plot(velocidadSalida,velocidadIzq,'b', linewidth=1.5, label = 'Izquierda')
	# graficavelocidadSalida.plot(velocidadSalida,velocidadCen_Izq, 'r', linewidth=1.5, label = 'Centro-Izq')
	# graficavelocidadSalida.plot(velocidadSalida,velocidadCen, 'm', linewidth=1.5, label = 'Centro')
	# graficavelocidadSalida.plot(velocidadSalida,velocidadCen_Der, 'y', linewidth=1.5, label = 'Centro-Der')
	# graficavelocidadSalida.plot(velocidadSalida,velocidadDer, 'g', linewidth=1.5, label = 'Derecha')
	# graficavelocidadSalida.set_title('Velocidad y Sentido salida')
	# graficavelocidadSalida.legend()
	
	# plt.tight_layout()
	
	# CALCULAMOS LOS VALORES DEL BORROSO PARA DESPUES TOMAR UNA DECISION 
	nivelAnguloActEntranteIzq = fuzzy.interp_membership(anguloActual, entActIzq, desviacionActual)
	nivelAnguloActEntranteCen = fuzzy.interp_membership(anguloActual, entActCen, desviacionActual)
	nivelAnguloActEntranteDer = fuzzy.interp_membership(anguloActual, entActDer, desviacionActual)
	
	nivelErrorEntranteIzq = fuzzy.interp_membership(errorDesviacion, entErrorIzq, error)
	nivelErrorEntranteCen = fuzzy.interp_membership(errorDesviacion, entErrorCen, error)
	nivelErrorEntranteDer = fuzzy.interp_membership(errorDesviacion, entErrorDer, error)
	
	# APLICAMOS LAS REGLAS
	
	# REGLA 1: IF error = cen AND actual = cen THEN salida = cen
	rule1 = np.fmin(nivelErrorEntranteCen, nivelAnguloActEntranteCen)
	rule1Cen = np.fmin(rule1, velocidadCen)
	
	# REGLA 2: IF error = cen AND actual = pos THEN salida = cen-der
	rule2 = np.fmin(nivelErrorEntranteCen, nivelAnguloActEntranteDer)
	rule2Cen_Der = np.fmin(rule2, velocidadCen_Der)
	
	# REGLA 3: IF error = cen AND actual = neg THEN salida = cen-izq
	rule3 = np.fmin(nivelErrorEntranteCen, nivelAnguloActEntranteIzq)
	rule3Cen_Izq = np.fmin(rule3, velocidadCen_Izq)
	
	# REGLA 4: IF error = pos AND actual = cen THEN salida = cen-der
	rule4 = np.fmin(nivelErrorEntranteDer, nivelAnguloActEntranteCen)
	rule4Cen_Der = np.fmin(rule4, velocidadCen_Der)
	
	# REGLA 5: IF error = pos AND actual = pos THEN salida = der
	rule5 = np.fmin(nivelErrorEntranteDer, nivelAnguloActEntranteDer)
	rule5Der = np.fmin(rule5, velocidadDer)
	
	# REGLA 6: IF error = neg AND actual = cen THEN salida = cen-izq
	rule6 = np.fmin(nivelErrorEntranteIzq, nivelAnguloActEntranteCen)
	rule6Cen_Izq = np.fmin(rule6, velocidadCen_Izq)
	
	# REGLA 7: IF error = neg AND actual = neg THEN salida = izq
	rule7 = np.fmin(nivelErrorEntranteIzq, nivelAnguloActEntranteIzq)
	rule7Cen_Izq = np.fmin(rule7, velocidadCen_Izq)

	vel0 = np.zeros_like(velocidadSalida)
	
	reglaInter1 = np.fmax(rule2Cen_Der, rule3Cen_Izq)
	reglaInter2 = np.fmax(rule4Cen_Der, rule5Der)
	reglaInter3 = np.fmax(rule6Cen_Izq, rule7Cen_Izq)
	
	reglaInter4 = np.fmax(reglaInter1, reglaInter2)
	# Agregacion de las funciones de pertenencia de salida
	aggregated = np.fmax(rule1Cen, np.fmax(reglaInter3, reglaInter4))

	# Calculate defuzzified result
	velS = fuzzy.defuzz(velocidadSalida, aggregated, 'centroid')

	# plt.show
	
	return velS

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
	
def ejecucionBorroso():
	
	global error, desviacionActual, desviacionAnterior, velocidad, valoresUltra
	
	error, desviacionAnterior, desviacionActual = calculoRepulsion(valoresUltra,error, desviacionAnterior, desviacionActual)
	velocidad = borroso(error, desviacionActual)
	
	return velocidad
	
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def enviarVelocidad(vel):
		
	velIntermedia =  int(vel*255) # Regla de tres para apliar bien los valores
	
	velocidadStr = str(velIntermedia*(10))
	print("Se envia la velocidad: ")
	print(velocidadStr)
	client.publish("robot",velocidadStr)
	
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

print('__--__ Se ejecuta el Cliente Borroso __--__')

print("Creando instancia del Cliente Borroso")
client = mqtt.Client("P1") 

print("Conectando con el Broker")
client.connect(ipBroker)

# Se recoge si hay algun topic que ha recibido informacion

print("Subscribiendose al topic","robot/distancia/u_izq")
client.subscribe("robot/distancia/u_izq")
print("Subscribiendose al","robot/distancia/u_cen")
client.subscribe("robot/distancia/u_cen")
print("Subscribiendose al topic","robot/distancia/u_der")
client.subscribe("robot/distancia/u_der")
print("Subscribiendose al topic","robot/dht/temp")
client.subscribe("robot/dht/temp")
print("Subscribiendose al topic","robot/dht/hum")
client.subscribe("robot/dht/hum")
print("Subscribiendose al topic","robot/posicion/x")
client.subscribe("robot/posicion/x")
print("Subscribiendose al topic","robot/posicion/y")
client.subscribe("robot/posicion/y")
print("Subscribiendose al topic","robot/mapa")
client.subscribe("robot/mapa")

client.on_message = on_message

client.publish("robot", "Comenzar ejecucion")

client.loop_forever()

print ('__--__ Fin de la ejecucion del Cliente Borroso __--__')
