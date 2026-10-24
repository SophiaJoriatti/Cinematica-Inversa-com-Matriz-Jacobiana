# Cinemática Inversa com Matriz Jacobiana (ABB 3DoF e LWA4 7DoF)

Este projeto implementa a cinemática direta e inversa para dois manipuladores robóticos (um ABB de 3 graus de liberdade e um Schunk LWA4 de 7 graus de liberdade) utilizando MATLAB/Octave. O cálculo da cinemática inversa é feito dinamicamente através do método da **Pseudo-inversa do Jacobiano com amortecimento (Regularização de Levenberg-Marquardt)** para seguir trajetórias 3D no espaço[cite: 21, 22, 23, 24, 25, 26, 27].

## 🛠️ Estrutura do Projeto

Ambos os scripts partilham uma arquitetura de simulação semelhante, dividida nas seguintes etapas:
1. **Modelagem Simbólica (D-H):** Cálculo da cinemática direta genérica e transformação das matrizes para o efetor final.
2. **Cálculo do Jacobiano Analítico:** Derivação da matriz Jacobiana em relação às variáveis de junta.
3. **Pré-computação Numérica (`matlabFunction`):** Conversão das matrizes simbólicas em funções numéricas nativas para acelerar o tempo de simulação no loop de controlo.
4. **Controlo e Cinemática Inversa:** Loop de otimização que calcula o erro cartesiano (GradEQM) e atualiza os ângulos das juntas (`Theta`) utilizando a pseudo-inversa amortecida para evitar singularidades matemáticas.
5. **Animação 3D:** Plotagem iterativa dos frames, hastes de ligação e rasto da trajetória no espaço tridimensional.

## 🤖 Robôs Simulados

### 1. Manipulador ABB (3 DoF) - `abb_3dof.m`
Robô de 3 juntas rotacionais[cite: 21]. O script traça uma trajetória tridimensional parabólica/circular (combinação de cosseno, seno e função quadrática) no plano X-Y-Z[cite: 22]. A interface plota marcadores em cores diferentes (verde, vermelho, azul e ciano) para representar as juntas base, ombro, cotovelo e o efetor final[cite: 23].

### 2. Manipulador Schunk LWA4 (7 DoF) - `lwa4_7dof.m`
Robô redundante de 7 graus de liberdade rotacionais[cite: 24]. Dada a sua redundância cinemática (mais graus de liberdade do que os necessários para posicionamento espacial), o algoritmo do Jacobiano amortecido brilha ao encontrar uma solução suave para a trajetória[cite: 25, 26]. A animação mostra uma cadeia longa de sete *frames* coloridos rastreando a mesma trajetória parabólica[cite: 26, 27].

## 🚀 Como Executar

1. Necessita do **GNU Octave** com o pacote `symbolic` instalado (ou MATLAB).
2. Execute os ficheiros `.m` diretamente. A consola exibirá o progresso da conversão das funções numéricas (o que pode demorar alguns segundos) e o erro a cada iteração, abrindo de seguida a animação 3D do rastreamento.
