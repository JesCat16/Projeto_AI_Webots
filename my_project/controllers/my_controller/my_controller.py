# Refactored: CamelCase with Hungarian notation following Python conventions
import math

from controller import Supervisor

iBoxCount = 5

def boxMovedSignificantly(tplBefore, tplAfter, fThreshold=0.02):
    """Retorna True se a caixa se moveu mais que o limite especificado"""
    fDistance = getEuclidianDistance(tplBefore, tplAfter)
    return fDistance > fThreshold

def wait(supSupervisor, iDurationMs):
    fStart = supSupervisor.getTime()
    while supSupervisor.getTime() - fStart < iDurationMs / 1000.0:
        supSupervisor.step(450)


# Calcula a distância euclidiana entre dois pontos 2D
def getEuclidianDistance(tplPosA, tplPosB):
    fXDistance = (tplPosB[0] - tplPosA[0])
    fYDistance = (tplPosB[1] - tplPosA[1])
    return math.hypot(fXDistance, fYDistance)


def printBoxPositions(lstBoxes):
    print("\nBox positions:")
    for iIndex, nodBox in enumerate(lstBoxes):
        tplPos = nodBox.getPosition()
        print(f"C{iIndex}: X={tplPos[0]:.2f}, Y={tplPos[1]:.2f}")


# Inicializa as caixas e registra suas posições iniciais
def initializeBoxes(supSupervisor):
    lstBoxes = []
    dictInitialPositions = {}
    for iIdx in range(iBoxCount):
        sDefName = f"C{iIdx}"
        nodBox = supSupervisor.getFromDef(sDefName)
        if nodBox:
            lstBoxes.append(nodBox)
            dictInitialPositions[sDefName] = tuple(nodBox.getPosition()[0:2])
    return lstBoxes, dictInitialPositions


# Ativa e retorna os sensores de proximidade
def initializeSensors(supSupervisor):
    lstSensors = []
    for i in range(8):
        sensor = supSupervisor.getDevice(f"ps{i}")
        sensor.enable(450)
        lstSensors.append(sensor)
    return lstSensors


def readSensorValues(lstSensors):
    return [sen.getValue() for sen in lstSensors]


# Calcula a direção que o robô precisa ajustar para apontar corretamente para o alvo
def computeOrientationError(nodRobot, nodTarget):
    # Posição do robô (x, y, z) — só usaremos x e y
    lstRobotPosition = nodRobot.getField("translation").getSFVec3f()
    fXRobot = lstRobotPosition[0]
    fYRobot = lstRobotPosition[1]

    # Posição da caixa
    fXBox, fYBox = nodTarget.getPosition()[0:2]

    # Calcula a direção desejada até o alvo
    fDeltaX = fXBox - fXRobot
    fDeltaY = fYBox - fYRobot
    # Angulo ideal para que ele chegue na caixa de forma reta
    fDesiredAngle = math.atan2(fDeltaY, fDeltaX)

    # Ângulo atual do robô baseado na sua rotação
    robotRotation = nodRobot.getField("rotation").getSFRotation()
    iCurrentAngle = robotRotation[3] * (1 if robotRotation[2] >= 0 else -1)

    # Calcula o menor erro angular entre a direção atual e a desejada
    fRawError = fDesiredAngle - iCurrentAngle

    # Normaliza o erro para o intervalo [-pi, pi], garantindo o menor giro possível
    angleError = (fRawError + math.pi) % (2 * math.pi) - math.pi

    # Distância entre robô a caixa
    distance = getEuclidianDistance((fXRobot, fYRobot), (fXBox, fYBox))

    return angleError, distance


# Faz o robô se mover em direção ao alvo, desviando de obstáculos e ajustando a direção
def navigateToTarget(nodRobot, nodTarget, motLeft, motRight, supSupervisor):
    # TODO FALTA LOGICA DE DESVIAR DAS CAIXAS
    # Leitura dos sensores
    lstSensorValues = readSensorValues(lstSensors)

    # Detecta obstáculo à frente (sensores frontais)
    isObstacleFront = lstSensorValues[0] > 80 or lstSensorValues[1] > 80 or lstSensorValues[6] > 80 or lstSensorValues[7] > 80

    if isObstacleFront:
        #print("[EVASÃO] Obstáculo à frente detectado! Desviando...")

        # Gira no lugar para desviar
        motLeft.setVelocity(-2.5)
        motRight.setVelocity(2.5)

        wait(supSupervisor, 1000)

        motLeft.setVelocity(0)
        motRight.setVelocity(0)

        return  # Sai da função para não continuar a navegação
    # Calcula o erro de direção e a distância até o alvo
    directionError, distanceToTarget = computeOrientationError(nodRobot, nodTarget)

    # Se estiver muito desalinhado, gira no lugar para se alinhar
    if abs(directionError) > 2.5:
        rotationSpeed = 2.5 if directionError > 0 else -2.5
        motLeft.setVelocity(-rotationSpeed)
        motRight.setVelocity(rotationSpeed)
        wait(supSupervisor, 2000)
        return

    # Controlador proporcional para ajuste suave de direção
    gain = 0.6 * (1 + distanceToTarget)
    baseSpeed = 5.0
    correction = max(min(gain * directionError, baseSpeed), -baseSpeed)

    # Define as velocidades das rodas com base no ajuste
    leftSpeed = max(min(baseSpeed - correction, 6.28), -6.28)
    rightSpeed = max(min(baseSpeed + correction, 6.28), -6.28)

    print(f"[NAVIGATION] Erro angular: {directionError:.2f} rad, Distância até alvo: {distanceToTarget:.2f} m")
    print(f"[NAVIGATION] Velocidades - Esquerda: {leftSpeed:.2f}, Direita: {rightSpeed:.2f}")

    motLeft.setVelocity(leftSpeed)
    motRight.setVelocity(rightSpeed)


# Verifica se o robô chegou suficientemente perto do alvo (caixa)
def hasArrived(nodRobot, nodTarget):
    # Obtém a posição atual do robô
    tplRobotPos = nodRobot.getField("translation").getSFVec3f()
    fXRobot = tplRobotPos[0]
    fYRobot = tplRobotPos[1]

    # Obtém a posição da caixa
    fXTarget, fYTarget = nodTarget.getPosition()[0:2]

    # Calcula a distância entre o robô e a caixa
    fDistance = getEuclidianDistance((fXRobot, fYRobot), (fXTarget, fYTarget))

    print(f"[CHECK DISTANCE] Distância até caixa: {fDistance:.4f} m")

    return fDistance <= 0.10


# Simula o empurrão da caixa pelo robô durante um tempo definido
def pushBox(supSupervisor, motLeft, motRight):
    print("Iniciando empurrão da caixa...")

    # Define a velocidade máxima para ambos os motores
    fMaxVelocity = 6.28
    motLeft.setVelocity(fMaxVelocity)
    motRight.setVelocity(fMaxVelocity)

    iStepsToPushTheBox = 2

    # Executa os ciclos mantendo a velocidade
    for _ in range(iStepsToPushTheBox):
        supSupervisor.step(450)

    # Para os motores após o empurrão
    motLeft.setVelocity(0)
    motRight.setVelocity(0)


def getNearestBox(nodRobot, lstBoxes):
    # Obtém a posição atual do robô (x, y, z)
    tplRobotPos = nodRobot.getField("translation").getSFVec3f()
    xRobot = tplRobotPos[0]
    yRobot = tplRobotPos[1]

    # Inicializa as variáveis de controle da menor distância
    iNearestBoxIndex = -1
    nodNearestBox = None
    fMinDistance = float('inf')

    # Itera por todas as caixas para encontrar a mais próxima
    for iIndex, nodBox in enumerate(lstBoxes):
        # Obtém a posição da caixa (x, y)
        fXBoxPosition, fYBoxPosition = nodBox.getPosition()[0:2]

        # Calcula a distância entre o robô e a caixa
        fDistance = getEuclidianDistance((xRobot, yRobot), (fXBoxPosition, fYBoxPosition))

        # Atualiza o mais próximo, se necessário
        if fDistance < fMinDistance:
            fMinDistance = fDistance
            iNearestBoxIndex = iIndex
            nodNearestBox = nodBox

    return iNearestBoxIndex, nodNearestBox

def rotateRobotInPlace(motLeft, motRight, supSupervisor, fSeconds=1.5, fSpeed=2.5):
    print("[ROBOT] Caixa leve detectada. Girando no próprio eixo...")
    for i in range(2):
        motLeft.setVelocity(-fSpeed)
        motRight.setVelocity(fSpeed)

        wait(supSupervisor, int(fSeconds * 1000))

        motLeft.setVelocity(0)
        motRight.setVelocity(0)

supSupervisor = Supervisor()
motLeft = supSupervisor.getDevice("left wheel motor")
motRight = supSupervisor.getDevice("right wheel motor")
motLeft.setPosition(float('inf'))
motRight.setPosition(float('inf'))
lstSensors = initializeSensors(supSupervisor)
robot = supSupervisor.getFromDef("ROBO")

lstBoxes, dictInitialPositions = initializeBoxes(supSupervisor)
lstRemainingBoxes = lstBoxes.copy()

# Loop principal do controle do robô
while supSupervisor.step(450) != -1:

    if len(lstRemainingBoxes) == 0:
        break

    iNearestBoxIndex, nodNearestBox = getNearestBox(robot, lstRemainingBoxes)
    if hasArrived(robot, nodNearestBox):
        
        sBoxId = f"C{iNearestBoxIndex}"
        tplStartPos = dictInitialPositions[sBoxId]
        
        tplPrePushPos = nodNearestBox.getPosition()[0:2]
        
        pushBox(supSupervisor, motLeft, motRight)
        
        tplEndPos = nodNearestBox.getPosition()[0:2]
        
        bMoved = boxMovedSignificantly(tplPrePushPos, tplEndPos)
        
        fMovedDistance = getEuclidianDistance(tplStartPos, tplEndPos)
        
        lstRemainingBoxes.pop(iNearestBoxIndex)
        if bMoved:
            rotateRobotInPlace(motLeft, motRight, supSupervisor)
            print(f"[INFO] Caixa leve encontrada. Parando execução.")
            break
            
        print(f"Finalizou empurrar, indo para a próxima caixa")
    else:
        navigateToTarget(robot, nodNearestBox, motLeft, motRight, supSupervisor)

#print("\n[RESULTADO FINAL] Análise do deslocamento das caixas:")
for iIdx, nodBox in enumerate(lstBoxes):
    sBoxId = f"C{iIdx}"
    tplStartPosition = dictInitialPositions[sBoxId]
    tplEndPosition = nodBox.getPosition()[0:2]
    fMovedDistance = getEuclidianDistance(tplStartPosition, tplEndPosition)
    sBoxStatus = "LEVE" if fMovedDistance > 0.001 else "PESADA"
    if fMovedDistance <= 0.001 and sBoxStatus == "LEVE":
        print(f"     [ALERTA] A caixa {sBoxId} foi marcada como LEVE, mas não se moveu!")
    #print(f" - {sBoxId}")
    #print(f"     Posição inicial: X={tplStartPosition[0]:.2f}, Y={tplStartPosition[1]:.2f}")
    #print(f"     Posição final  : X={tplEndPosition[0]:.2f}, Y={tplEndPosition[1]:.2f}")
    #print(f"     Deslocamento   : {fMovedDistance:.4f} metros")
    #print(f"     Resultado      : {sBoxStatus}")
