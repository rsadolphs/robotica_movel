# 🔧 Guia Técnico - Mini-Mapa Radar

## Estruturas de Dados

### `RadarCoord`
```cpp
struct RadarCoord {
    float x;                  // Posição X no radar
    float y;                  // Posição Y no radar
    bool outOfBounds;         // true se projetado na borda
};
```

### `RadarPoint` (em Rendering.hpp)
```cpp
struct RadarPoint {
    float x;
    float y;
    Color color;
    float radius;  // Raio para desenho
};
```

## Funções Principais

### 1. `convertToRadarCoords()`

**Assinatura:**
```cpp
RadarCoord convertToRadarCoords(
    float centroidX, float centroidY,      // Posição global do centroide
    float robotX, float robotY,            // Posição global do robô
    float robotTheta,                      // Orientação do robô (radianos)
    float radarRadius                      // Raio máximo do radar (metros)
);
```

**Comportamento:**
```cpp
// Passo 1: Translação
float relX = centroidX - robotX;
float relY = centroidY - robotY;

// Passo 2: Rotação pela orientação do robô
// Transforma para frame onde robô aponta para (0, 1)
float cosTheta = std::cos(robotTheta);
float sinTheta = std::sin(robotTheta);
float rotatedX = relX * cosTheta + relY * sinTheta;
float rotatedY = -relX * sinTheta + relY * cosTheta;

// Passo 3: Distância e ângulo
float distance = std::sqrt(rotatedX * rotatedX + rotatedY * rotatedY);
float angle = std::atan2(rotatedX, rotatedY);

// Passo 4: Projeção
if (distance > radarRadius) {
    // Fora do raio: projeta na borda mantendo ângulo
    result.x = radarRadius * std::sin(angle);
    result.y = radarRadius * std::cos(angle);
    result.outOfBounds = true;
} else {
    // Dentro do raio: escala linear
    float scale = distance / radarRadius;
    result.x = (radarRadius * scale) * std::sin(angle);
    result.y = (radarRadius * scale) * std::cos(angle);
    result.outOfBounds = false;
}
```

### 2. `drawRadar()`

**Assinatura:**
```cpp
void drawRadar(float radarCenterX, float radarCenterY, float radarRadius);
```

**Elementos Desenhados:**

```
1. Background circular (GL_TRIANGLE_FAN)
   ├─ Cor: (0.2, 0.2, 0.3) - Cinza escuro
   └─ 100 segmentos

2. Borda do radar (círculo)
   ├─ Cor: (0.0, 1.0, 0.0) - Verde
   ├─ Raio: radarRadius
   └─ 100 segmentos

3. Círculos de referência
   ├─ 50% do raio: Cor (0.3, 0.5, 0.3)
   └─ 33% do raio: Cor (0.3, 0.5, 0.3)

4. Marcadores de direção (cruz)
   ├─ Vertical (forward): Y=+0.9*radarRadius
   └─ Horizontal (left): X=-0.9*radarRadius

5. Robô (triângulo)
   ├─ Cor: (1.0, 1.0, 0.0) - Amarelo
   ├─ Tamanho: 0.2-0.3 unidades
   └─ Ponta aponta para frente (+Y)

6. Centroides dos clusters
   ├─ Próximos (outOfBounds=false)
   │  ├─ Cor: (0.0, 1.0, 1.0) - Cyan
   │  ├─ Tamanho: 0.25 unidades
   │  └─ Sem contorno
   └─ Distantes (outOfBounds=true)
      ├─ Cor: (1.0, 0.0, 0.0) - Vermelho
      ├─ Tamanho: 0.30 unidades
      └─ Contorno branco (1.0, 1.0, 1.0)
```

### 3. `drawCircle()`

**Assinatura:**
```cpp
void drawCircle(float centerX, float centerY, float radius, int segments = 100);
```

**Implementação:**
```cpp
glBegin(GL_LINE_LOOP);
for (int i = 0; i < segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float x = centerX + radius * std::cos(angle);
    float y = centerY + radius * std::sin(angle);
    glVertex2f(x, y);
}
glEnd();
```

### 4. `drawPoint()`

**Assinatura:**
```cpp
void drawPoint(float x, float y, float size, float r, float g, float b);
```

**Desenha um quadrado:**
```cpp
glColor3f(r, g, b);
glBegin(GL_QUADS);
    glVertex2f(x - size/2, y - size/2);
    glVertex2f(x + size/2, y - size/2);
    glVertex2f(x + size/2, y + size/2);
    glVertex2f(x - size/2, y + size/2);
glEnd();
```

## Loop de Renderização

### Estrutura Principal

```cpp
void* renderingThreadFunction(void* arg) {
    // 1. Inicializar GLFW (uma única vez)
    if (!glfwInit()) return NULL;
    
    // 2. Criar janelas (mapa + radar)
    GLFWwindow* window = glfwCreateWindow(600, 600, "Mapping", NULL, NULL);
    GLFWwindow* radarWindow = glfwCreateWindow(500, 500, "Radar", NULL, window);
    
    // 3. Loop de renderização
    while (!glfwWindowShouldClose(window) && 
           !glfwWindowShouldClose(radarWindow)) {
        
        // 3.1. Renderizar Mapa Principal
        glfwMakeContextCurrent(window);
        // ... configurar OpenGL ...
        // ... desenhar células ...
        glfwSwapBuffers(window);
        
        // 3.2. Renderizar Radar
        glfwMakeContextCurrent(radarWindow);
        // ... configurar OpenGL ...
        drawRadar(...);
        glfwSwapBuffers(radarWindow);
        
        // 3.3. Processar eventos
        glfwPollEvents();
        
        // 3.4. Controlar FPS
        usleep(50000);  // 20 Hz
    }
    
    // 4. Limpar
    glfwDestroyWindow(window);
    glfwDestroyWindow(radarWindow);
    glfwTerminate();
    return NULL;
}
```

## Transformações Matemáticas

### Matriz de Rotação (2D)

Para rotacionar um ponto (x, y) pelo ângulo θ:

```cpp
float cosTheta = std::cos(theta);
float sinTheta = std::sin(theta);

float x_rotated = x * cosTheta - y * sinTheta;
float y_rotated = x * sinTheta + y * cosTheta;
```

No caso do radar, usando rotação por theta do robô:
```cpp
// Rotaciona para frame onde robô aponta para (0, 1)
// Nota: Usa -sin ao invés de sin em uma das linhas
float x_rotated = relX * cosTheta + relY * sinTheta;
float y_rotated = -relX * sinTheta + relY * cosTheta;
```

### Conversão Polar (Cartesiano → Polar)

```cpp
// De (x, y) para (distância, ângulo)
float distance = std::sqrt(x*x + y*y);
float angle = std::atan2(x, y);  // atan2(y, x) no geral, mas aqui atan2(x, y)

// Inverso: de (distância, ângulo) para (x, y)
float x = distance * std::sin(angle);
float y = distance * std::cos(angle);
```

## OpenGL Context Management

### Alternar Contextos

```cpp
// Fazer contexto 1 ativo
glfwMakeContextCurrent(window1);
glClear(GL_COLOR_BUFFER_BIT);
glfwSwapBuffers(window1);

// Fazer contexto 2 ativo
glfwMakeContextCurrent(window2);
glClear(GL_COLOR_BUFFER_BIT);
glfwSwapBuffers(window2);
```

### Estados OpenGL

Cada contexto mantém seu próprio estado de:
- Matriz de projeção (GL_PROJECTION)
- Matriz de visualização (GL_MODELVIEW)
- Cores e atributos
- Viewport

## Exemplo: Adicionar um Novo Centroide

```cpp
// 1. Obter informações do centroide
FrontierCluster cluster = clusters[i];
float centroidX = cluster.centroidX;
float centroidY = cluster.centroidY;

// 2. Converter para coordenadas radar
RadarCoord radarCoord = convertToRadarCoords(
    centroidX, centroidY,
    robotPosition.x, robotPosition.y,
    robotPosition.theta,
    radarRadius
);

// 3. Calcular posição na tela
float displayX = radarCenterX + radarCoord.x;
float displayY = radarCenterY + radarCoord.y;

// 4. Desenhar
if (radarCoord.outOfBounds) {
    // Fora: vermelho com contorno
    drawPoint(displayX, displayY, 0.3f, 1.0f, 0.0f, 0.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINE_LOOP);
        // ... desenhar contorno ...
    glEnd();
} else {
    // Dentro: cyan
    drawPoint(displayX, displayY, 0.25f, 0.0f, 1.0f, 1.0f);
}
```

## Debug: Imprimir Dados do Radar

```cpp
// Adicionar após convertToRadarCoords()
#ifdef DEBUG_RADAR
std::cout << "Centroide: (" << centroidX << ", " << centroidY << ")" << std::endl;
std::cout << "Robô: (" << robotPosition.x << ", " << robotPosition.y << ")" << std::endl;
std::cout << "Theta: " << robotPosition.theta << std::endl;
std::cout << "Posição Radar: (" << radarCoord.x << ", " << radarCoord.y << ")" << std::endl;
std::cout << "Out of Bounds: " << (radarCoord.outOfBounds ? "SIM" : "NÃO") << std::endl;
#endif
```

## Performance Otimização

### Reduzir FPS (se CPU alto)
```cpp
usleep(100000);  // 10 Hz ao invés de 20 Hz
```

### Cache de Clusters
```cpp
// Adicionar variável global
static std::vector<FrontierCluster> cachedClusters;
static uint64_t lastUpdateTime = 0;

// No drawRadar():
uint64_t now = std::chrono::system_clock::now().time_since_epoch().count();
if (now - lastUpdateTime > 100000000) {  // Atualizar a cada 100ms
    cachedClusters = detectFrontierClusters();
    lastUpdateTime = now;
}

// Usar cachedClusters ao invés de chamar detectFrontierClusters()
for (const auto& cluster : cachedClusters) {
    // ...
}
```

---

**Última Atualização**: 2026-06-20  
**Versão**: 1.0
