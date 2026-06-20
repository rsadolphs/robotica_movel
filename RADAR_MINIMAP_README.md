# Mini-Mapa Radar 🎯

## Visão Geral
Uma segunda janela de visualização foi adicionada ao seu sistema de navegação. Ela funciona como um **radar/mini-mapa em tempo real**, mostrando a posição do robô e os centroides dos clusters de fronteira ao seu redor.

## Como Funciona

### Layout do Radar
```
        ↑ (Forward)
        |
    ◊---+---◊     ◊ = Marcadores de distância
    |   |   |
    +---R---+     R = Robô (centro, triângulo amarelo)
    |       |
    ◊-------◊     Círculo verde = limite do radar
```

### Características Principais

1. **Robô no Centro**
   - Representado por um triângulo amarelo
   - Sempre fixo no centro da janela
   - Aponta para a direção "forward" do robô

2. **Círculos de Referência**
   - Círculo verde externo: raio máximo do radar (10 metros)
   - Círculos internos tracejados: marcadores de distância (50% e 33% do raio)

3. **Centroides de Clusters**
   - **Cyan (azul claro)**: Centroides dentro do raio do radar
     - Posicionados com escala linear de distância
   - **Vermelho com contorno branco**: Centroides fora do raio do radar
     - Projetados na borda do círculo
     - Ângulo real preservado em relação ao robô

4. **Orientação**
   - A linha vertical aponta para frente (forward direction)
   - A linha horizontal aponta para esquerda
   - O sistema de coordenadas está sempre alinhado com o robô

## Transformações de Coordenadas

### Algoritmo de Conversão
```
1. Translada centroide para frame do robô
2. Rotaciona usando theta do robô
3. Calcula distância e ângulo
4. Se distância ≤ radarRadius:
     → Escala linear: (distância / radarRadius)
5. Se distância > radarRadius:
     → Projeta na borda mantendo ângulo
```

### Exemplo
Se um centroide está:
- A 15 metros de distância (raio = 10m): **Aparece na borda com ângulo correto**
- A 5 metros de distância: **Aparece a 50% da borda**
- A 2 metros de distância: **Aparece a 20% da borda**

## Visualização

### Janelas
- **Mapa Principal** (600x600): Visão global do ambiente
- **Radar/Mini-mapa** (500x500): Visão localizada ao redor do robô

### Cores
| Elemento | Cor | RGB |
|----------|-----|-----|
| Robô | Amarelo | (1.0, 1.0, 0.0) |
| Círculo Radar | Verde | (0.0, 1.0, 0.0) |
| Centroide Próximo | Cyan | (0.0, 1.0, 1.0) |
| Centroide Distante | Vermelho | (1.0, 0.0, 0.0) |
| Background | Escuro | (0.15, 0.15, 0.15) |
| Grid Referência | Verde Escuro | (0.3, 0.5, 0.3) |

## Performance

- **Taxa de Renderização**: ~20 Hz
- **Atualização de Clusters**: A cada frame
- **Gerenciamento de Memória**: Uma única thread gerencia ambas as janelas
- **Context Switching**: Alternância eficiente entre contextos OpenGL

## Configurações

### Parâmetros Ajustáveis (em Rendering.cpp)
```cpp
float radarRadius = 10.0f;      // Raio do radar em metros
usleep(50000);                  // Período de renderização (50ms = 20Hz)
int radarWidth = 500;           // Largura da janela radar
int radarHeight = 500;          // Altura da janela radar
```

## Arquivos Modificados

### Headers
- `include/Rendering.hpp`: Estruturas e declarações

### Implementação
- `src/Rendering.cpp`: Lógica de renderização e conversão de coordenadas

### Novas Funções
- `drawCircle()`: Desenha círculos no radar
- `drawPoint()`: Desenha pontos (centroides)
- `convertToRadarCoords()`: Converte coordenadas cartesianas para radar
- `drawRadar()`: Renderiza o radar completo

## Futuras Melhorias

1. **Cache de Clusters**: Armazenar clusters para evitar cálculos repetitivos
2. **Zoom Dinâmico**: Permitir ajuste do raio do radar em tempo real
3. **Zoom Automático**: Adaptar raio baseado na proximidade de centroides
4. **Histórico de Movimento**: Rastrear trajeto do robô
5. **Configuração Interativa**: Ajustar parâmetros via interface

## Troubleshooting

### Janela Radar não aparece
- Verifique se GLFW está instalado corretamente
- Certifique-se de que o display está configurado para múltiplas janelas

### Performance baixa
- Reduza a frequência de atualização aumentando o valor de `usleep`
- Considere usar cache para clusters

### Centroides não aparecem
- Verifique se `detectFrontierClusters()` está retornando dados
- Certifique-se de que `robotPosition` está sendo atualizado
