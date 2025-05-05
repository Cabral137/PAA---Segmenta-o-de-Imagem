#include <opencv2/opencv.hpp>   // Biblioteca OpenCV para manipulação de imagens
#include "Vertice.cpp"          // Arquivo que contém a definição da classe Vertice
#include <iostream>             // Para entrada e saída de dados
#include <queue>                // Para usar a estrutura de fila na BFS
#include <cmath>                // Para operações matemáticas, como INFINITY
#include <map>                  // Para armazenar os pais e nós visitados

using namespace std;

/**
 * @brief Realiza a busca em largura (BFS) para encontrar um caminho de aumento no grafo residual.
 * 
 * A função BFS encontra um caminho aumentante no grafo residual, ou seja, um caminho de um vértice fonte
 * (s) até o vértice terminal (t), onde há capacidade residual positiva nas arestas.
 * Este caminho é utilizado para aumentar o fluxo na rede.
 * 
 * @param s Ponteiro para o vértice fonte.
 * @param t Ponteiro para o vértice terminal.
 * @param grafo Referência ao grafo representado como uma matriz de ponteiros de vértices.
 * @param caminho Vetor que armazenará o caminho de aumento encontrado (se existir).
 * @return true Se um caminho de aumento foi encontrado.
 * @return false Se não há caminho de aumento disponível.
 */
bool BFS(Vertice *s, Vertice *t, vector<vector<Vertice*>> &grafo, vector<Vertice*> &caminho) 
{
    map<Vertice*, Vertice*> parent;  // Mapa que armazena os pais dos vértices para reconstrução do caminho
    queue<Vertice*> q;              // Fila usada para a BFS
    q.push(s);

    // Mapa para rastrear os vértices visitados
    map<Vertice*, bool> visitado;

    // Inicializa todos os vértices como não visitados
    for (auto &linha : grafo) 
    {
        for (auto &vertice : linha) 
        {
            if (vertice) visitado[vertice] = false; // Marca como não visitado
        }
    }

    if(s->getX() != -1){
        visitado[s] = true; // Marca o vértice fonte como visitado
    } 

    // Inicia a BFS
    while (!q.empty()) 
    {
        Vertice *u = q.front(); // Obtém o próximo vértice da fila
        q.pop();

        // Itera sobre os vizinhos do vértice atual
        for (int i = 0; i < u->getListaVertices().size(); i++) 
        {
            Vertice *v = u->getListaVertices().at(i);

            // Verifica se o vizinho não foi visitado e se há capacidade residual positiva
            if (v->getX() != -1 && !visitado[v] && u->getListaPesos().at(i) > 0) 
            {
                parent[v] = u;   // Define o vértice atual como pai do vizinho
                visitado[v] = true;  // Marca o vizinho como visitado
                q.push(v);       // Adiciona o vizinho à fila para explorar

                // Se o vértice terminal foi encontrado, reconstrua o caminho
                if (v == t) 
                {
                    Vertice *curr = t;

                    // Reconstrói o caminho de t até s
                    while (curr != s) 
                    {
                        caminho.insert(caminho.begin(), curr); // Adiciona o vértice ao caminho
                        curr = parent[curr];                  // Segue o pai para reconstruir o caminho
                    }
                    
                    caminho.insert(caminho.begin(), s); // Adiciona o vértice fonte ao início do caminho
                    return true;
                }
            }
        }
    }

    return false;  // Não há caminho de aumento
}

/**
 * @brief Atualiza o grafo residual com base no caminho de aumento encontrado.
 * 
 * Esta função atualiza as capacidades das arestas do grafo residual, diminuindo a capacidade das arestas
 * no caminho aumentante (direção do fluxo) e aumentando a capacidade das arestas reversas (direção contrária ao fluxo).
 * 
 * @param caminho Vetor contendo o caminho de aumento (uma sequência de vértices).
 */
void updateFluxo(vector<Vertice*> caminho) 
{
    float FluxoMaximo = INFINITY;

    // Encontra o gargalo, ou seja, a capacidade mínima no caminho
    for (int i = 0; i < caminho.size() - 1; i++) 
    {
        Vertice *u = caminho[i];
        Vertice *v = caminho[i + 1];

        // Percorre as arestas de u para encontrar a capacidade mínima
        for (int j = 0; j < u->getListaVertices().size(); j++) 
        {
            if (u->getListaVertices().at(j) == v) 
            {
                FluxoMaximo = min(FluxoMaximo, u->getListaPesos().at(j)); // Gargalo no caminho
                break;
            }
        }
    }

    // Atualiza os fluxos no grafo residual
    for (int i = 0; i < caminho.size() - 1; i++) 
    {
        Vertice *u = caminho[i];
        Vertice *v = caminho[i + 1];

        // Diminui o fluxo na direção direta (u -> v)
        for (int j = 0; j < u->getListaVertices().size(); j++) 
        {
            if (u->getListaVertices().at(j) == v) 
            {
                u->setPesoArray(j, u->getListaPesos().at(j) - FluxoMaximo);  // Diminui a capacidade residual
                break;
            }
        }

        // Aumenta o fluxo na direção reversa (v -> u)
        for (int j = 0; j < v->getListaVertices().size(); j++) 
        {
            if (v->getListaVertices().at(j) == u) 
            {
                v->setPesoArray(j, v->getListaPesos().at(j) + FluxoMaximo);  // Aumenta a capacidade residual
                break;
            }
        }
    }
}

/**
 * @brief Calcula o fluxo máximo no grafo usando o algoritmo de Ford-Fulkerson.
 * 
 * Esta função usa o algoritmo de Ford-Fulkerson para calcular o fluxo máximo entre a fonte (s) e o terminal (t),
 * através da repetida aplicação da BFS para encontrar caminhos aumentantes.
 * 
 * @param s Ponteiro para o vértice fonte.
 * @param t Ponteiro para o vértice terminal.
 * @param grafo Referência ao grafo representado como uma matriz de ponteiros de vértices.
 */
void FluxoMaximo(Vertice *s, Vertice *t, vector<vector<Vertice*>> &grafo) 
{
    vector<Vertice*> caminho;

    // Continua até que nenhum caminho de aumento seja encontrado
    while (true) 
    {
        caminho.clear(); // Limpa o caminho da iteração anterior
    
        // Chama a função BFS para encontrar um caminho aumentante
        if (!BFS(s, t, grafo, caminho)) 
        {
            break; // Sai do loop se não houver mais caminho aumentante
        }
      
        // Atualiza os fluxos no grafo com base no caminho encontrado
        updateFluxo(caminho);
        cout << endl;
    }

    // Exibe que o cálculo do fluxo máximo foi concluído
    cout << "Cálculo do fluxo máximo concluído." << endl;
}

/**
 * @brief Realiza uma busca em profundidade (DFS) a partir de um vértice e marca os vértices alcançáveis.
 * 
 * A função realiza a DFS no grafo, marcando todos os vértices que podem ser alcançados a partir do vértice de origem.
 * A busca é realizada até que todos os vértices alcançáveis sejam visitados.
 * 
 * @param s Ponteiro para o vértice de origem da DFS.
 * @param visitado Mapa que rastreia quais vértices foram visitados durante a DFS.
 */
void DFS(Vertice *s, map<Vertice*, bool> &visitado)
{
    if(s == NULL)
    {
        return;  // Retorna se o vértice for nulo
    }
    
    visitado[s] = true; // Marca o vértice como visitado

    // Itera sobre os vizinhos do vértice s
    for(int i = 0; i < s->getListaVertices().size(); i++)
    {
        Vertice *vizinho = s->getListaVertices().at(i);

        if(vizinho == NULL)
        {
            return;  // Retorna se o vizinho for nulo
        }

        // Verifica se o vizinho ainda não foi visitado e se o fluxo residual é maior que zero
        if(!visitado[vizinho] && s->getListaPesos().at(i) > 0) 
        {
            DFS(vizinho, visitado);  // Chama a DFS recursivamente
        }
    }
}

/**
 * @brief Cria e salva a imagem segmentada com base nos vértices visitados.
 * 
 * Esta função segmenta a imagem alterando as cores dos pixels com base nos conjuntos de vértices visitados.
 * Pixels do objeto são destacados em azul e pixels do fundo são destacados em vermelho.
 * 
 * @param visitado Mapa que contém os vértices visitados.
 * @param grafo Grafo representando a imagem.
 * @param imagePath Caminho para a imagem original.
 */
void createImagemSegmentada(map<Vertice*, bool> visitado, vector<vector<Vertice*>> &grafo, string imagePath)
{
    // Carrega a imagem usando OpenCV
    cv::Mat imagem = cv::imread(imagePath, cv::IMREAD_COLOR);

    if (imagem.empty()) 
    {
        cout << "Erro ao carregar a imagem!" << endl;
        return;
    }

    // Para cada pixel da imagem, atribui um valor azulado ou avermelhado
    for (int i = 0; i < imagem.rows; i++) 
    {
        for (int j = 0; j < imagem.cols; j++) 
        {
            // Acessa o pixel (i, j) no formato BGR
            cv::Vec3b cor = imagem.at<cv::Vec3b>(i, j);
            
            // A média dos canais BGR para definir a intensidade do pixel
            uchar intensidade = (cor[0] + cor[1] + cor[2]) / 3;  // Média dos canais B, G, R

            // Verifica se o vértice correspondente ao pixel foi visitado
            if(visitado[grafo[i][j]] == true)
            {
                // Atribui cor azul para pixels do objeto
                imagem.at<cv::Vec3b>(i, j)[0] = 255; // Canal Azul
                imagem.at<cv::Vec3b>(i, j)[1] = 0;   // Canal Verde
                imagem.at<cv::Vec3b>(i, j)[2] = 0;   // Canal Vermelho
            }
            else
            {
                // Atribui cor vermelha para pixels de fundo
                imagem.at<cv::Vec3b>(i, j)[0] = 0;   // Canal Azul
                imagem.at<cv::Vec3b>(i, j)[1] = 0;   // Canal Verde
                imagem.at<cv::Vec3b>(i, j)[2] = 255; // Canal Vermelho
            }
        }
    }

    // Salva a imagem segmentada
    cv::imwrite("../Imagens/imagemSegmentada.png", imagem);
}

/**
 * @brief Realiza uma busca em largura (BFS) para verificar os vértices alcançáveis a partir de um vértice de origem.
 * 
 * A função realiza uma busca em largura (BFS) no grafo a partir de um vértice \( v_1 \) e retorna um mapa que
 * indica quais vértices são alcançáveis a partir de \( v_1 \). A busca é realizada apenas nas arestas com capacidade
 * residual positiva (fluxo maior que zero).
 * 
 * @param v1 Ponteiro para o vértice de origem da busca.
 * @return `map<Vertice*, bool>` Mapa indicando os vértices alcançáveis a partir de \( v_1 \), onde a chave é o vértice
 *         e o valor é `true` se o vértice foi visitado (alcançável), `false` caso contrário.
 */
map<Vertice*, bool> verificarConjuntos(Vertice *v1)
{
    map<Vertice*, bool> visitado; // Mapa para armazenar os vértices visitados
    vector<Vertice*> fila;        // Fila usada para a busca em largura

    fila.push_back(v1);           // Adiciona o vértice de origem à fila
    visitado[v1] = true;          // Marca o vértice de origem como visitado

    // Enquanto houver vértices na fila
    while(!fila.empty()){
        // Para cada vizinho do vértice atual
        for(int i = 0; i < fila.at(0)->getListaVertices().size(); i++){
            Vertice *vizinho = fila.at(0)->getListaVertices().at(i);

            // Verifica se o vizinho não foi visitado e se há capacidade residual positiva
            if((!visitado[vizinho]) && fila.at(0)->getListaPesos().at(i) > 0){
                visitado[vizinho] = true;  // Marca o vizinho como visitado
                fila.push_back(vizinho);    // Adiciona o vizinho à fila
            }
        }
        fila.erase(fila.begin()); // Remove o primeiro vértice da fila
    }

    return visitado; // Retorna o mapa de vértices visitados
}

/**
 * @brief Calcula e segmenta a imagem baseada no corte mínimo no grafo.
 * 
 * A função realiza uma busca a partir do vértice \( s \) para identificar todos os vértices alcançáveis. Em seguida,
 * ela utiliza essa informação para segmentar a imagem e destacar os objetos de interesse. O conjunto de vértices alcançáveis
 * será destacado em azul e os outros em vermelho.
 * 
 * @param s Ponteiro para o vértice fonte (início da busca).
 * @param t Ponteiro para o vértice sumidouro (não utilizado diretamente nesta função, mas geralmente usado no contexto de corte mínimo).
 * @param grafo Referência ao grafo que representa a rede de fluxo (com vértices e arestas).
 * @param imagePath Caminho da imagem que será segmentada, para aplicação da segmentação visual.
 */
void corteMinimo(Vertice *s, Vertice *t, vector<vector<Vertice*>> &grafo, string imagePath)
{
    // Realiza a busca a partir do vértice s para verificar quais vértices são alcançáveis
    map<Vertice*, bool> visitado = verificarConjuntos(s);

    cout << "Vértices alcançáveis a partir de s:" << endl;

    // Segmenta a imagem e destaca os vértices alcançáveis (em azul) e não alcançáveis (em vermelho)
    createImagemSegmentada(visitado, grafo, imagePath);
}