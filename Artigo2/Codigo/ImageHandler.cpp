#include "CaminhoAumento.cpp"   // Inclui o arquivo que contém a função FluxoMaximo
#include <iostream>             // Para entrada e saída de dados
#include <vector>               // Para armazenar os pontos de objeto e fundo
#include <chrono>

using namespace std;

const float SIGMA = 30;             // Valor de sigma para o cálculo da intensidade de borda

cv::Mat imagem;                     // A imagem carregada
cv::Mat imagem_gray;                // A imagem convertida para preto e branco (escala de cinza)
cv::Mat imagemGrafo;

cv::Point ponto_inicial;            // Ponto inicial para desenhar
bool desenhando = false;            // Flag para verificar se está desenhando

cv::Scalar cor_pincel(0, 0, 255);   // Cor inicial do pincel (vermelho)
bool cor = true;                    // Marcação de cor para armazenamento dos pixels
int raio_pincel = 4;                // Raio do pincel (tamanho do círculo)

Vertice* S = new Vertice();         // Ponto de origem (fonte)
Vertice* T = new Vertice();         // Ponto de destino (terminal)

vector <cv::Point> seedObjeto;      // Pontos Vermelhos
vector <cv::Point> seedFundo;       // Pontos Azuis

/**
 * @brief Função de callback para desenhar na imagem ao clicar e arrastar o mouse.
 * 
 * Essa função é chamada a cada evento do mouse. Ela permite ao usuário desenhar na imagem, alternando entre as cores
 * vermelho e azul para marcar pontos de interesse (por exemplo, objeto e fundo) usando o mouse.
 * 
 * @param evento Evento do mouse (pressionado, movido, solto).
 * @param x Coordenada X do mouse.
 * @param y Coordenada Y do mouse.
 * @param flags Flags do evento.
 * @param param Parâmetro adicional (não utilizado aqui).
 */
void drawCallback (int evento, int x, int y, int flags, void* param) 
{
    // Se o botão esquerdo do mouse for pressionado
    if (evento == cv::EVENT_LBUTTONDOWN) 
    {
        ponto_inicial = cv::Point(x, y);  // Armazenar o ponto inicial
        desenhando = true;                // Começar a desenhar
    }
    else
    { 
        if (evento == cv::EVENT_MOUSEMOVE && desenhando) // Se o mouse for movido enquanto o botão esquerdo estiver pressionado
        {
            // Adiciona no vetor de seeds correspondente a cor selecionada
            if(cor)
            {
                seedObjeto.push_back(cv::Point(x, y));
            }
            else
            {
                seedFundo.push_back(cv::Point(x, y));
            }

            // Desenhar um círculo no ponto atual
            cv::circle(imagem_gray, cv::Point(x, y), raio_pincel, cor_pincel, -1);
            cv::circle(imagem, cv::Point(x, y), raio_pincel, cor_pincel, -1); // Desenha nas duas imagens para que apareça na interface

            cv::imshow("Imagem com Desenho", imagem); // Mostrar a imagem com o desenho
        }
        else 
        {
            if (evento == cv::EVENT_LBUTTONUP) // Se o botão esquerdo do mouse for solto
            {
                desenhando = false;
            }
        }
    }
}

/**
 * @brief Função chamada para alternar a cor do pincel.
 * 
 * Essa função altera a cor do pincel entre vermelho e azul a cada vez que a tecla 't' é pressionada.
 */
void toggleColor() 
{
    if (cor_pincel == cv::Scalar(0, 0, 255)) 
    {
        cor = false;
        cor_pincel = cv::Scalar(255, 0, 0); // Azul
        std::cout << "Cor do pincel: Azul" << std::endl;
    } 
    else 
    {
        cor = true;
        cor_pincel = cv::Scalar(0, 0, 255); // Vermelho
        std::cout << "Cor do pincel: Vermelho" << std::endl;
    }
}

/**
 * @brief Função que carrega a imagem, permite desenhar nela e salva a versão modificada.
 * 
 * A função carrega a imagem, converte para escala de cinza, e permite ao usuário desenhar na imagem para marcar
 * regiões de interesse. As regiões desenhadas em vermelho representam o objeto e as regiões desenhadas em azul representam o fundo.
 * 
 * @param fileName Caminho do arquivo da imagem que será carregada.
 * @return A imagem em escala de cinza após a marcação.
 */
cv::Mat createMarkedImage (string fileName)
{
    // Carregar a imagem
    imagem = cv::imread(fileName);

    if(imagem.size().width * imagem.size().height > 1238400)
    {
        raio_pincel = raio_pincel * 18; // Ajusta o tamanho do pincel para imagens grandes
    }
    else
    {
        if(imagem.size().width * imagem.size().height < 6400)
        {
            raio_pincel = 1; // Ajusta o tamanho do pincel para imagens pequenas
        }
    }

    if (imagem.empty()) 
    {
        std::cerr << "Erro ao carregar a imagem!" << std::endl;
        return(cv::Mat());
    }

    // Converter a imagem para preto e branco (escala de cinza)
    cv::cvtColor(imagem, imagem_gray, cv::COLOR_BGR2GRAY);
    imagemGrafo = imagem_gray;
    // Converter a imagem em escala de cinza para um formato de 3 canais para manter a consistência no desenho
    cv::cvtColor(imagem_gray, imagem_gray, cv::COLOR_GRAY2BGR);

    // Criar a janela para mostrar a imagem
    cv::namedWindow("Imagem com Desenho", cv::WINDOW_NORMAL); // Cria uma janela redimensionável
    cv::resizeWindow("Imagem com Desenho", 1290, 960);  // Define um tamanho fixo de 1290x960
    cv::imshow("Imagem com Desenho", imagem);

    // Configurar a função de callback do mouse
    cv::setMouseCallback("Imagem com Desenho", drawCallback, NULL);

    while (true) 
    {
        // Aguardar por eventos de teclado
        char tecla = cv::waitKey(1);  // Espera por 1 ms para pegar a tecla pressionada

        // Trocar a cor do pincel
        if (tecla == 't') 
        {
            toggleColor();
        }

        // Se pressionar a tecla 'ESC', sair do programa
        if (tecla == 27) 
        {
            break;
        }
    }

    // Salvar a imagem alterada como um novo arquivo PNG
    cv::imwrite("../Imagens/imagemMarcada.png", imagem_gray);

    // Fechar todas as janelas
    cv::destroyAllWindows();

    return(imagemGrafo);
}

/**
 * @brief Calcula a intensidade de borda entre dois pixels da imagem.
 * 
 * A função calcula a intensidade de borda entre dois valores de pixel (v1 e v2), aplicando uma fórmula baseada na
 * diferença quadrática entre eles, ponderada por um fator \(\sigma\).
 * 
 * @param v1 Valor do primeiro pixel (intensidade do canal).
 * @param v2 Valor do segundo pixel (intensidade do canal).
 * @return A intensidade de borda calculada.
 */
float BorderIndex(u_char v1, u_char v2)
{
    if(v1 <= v2)
    {
        return 1.0;
    } 
    else 
    {
        return exp(-(((v1 - v2)*(v1 - v2))/(2.0*(SIGMA*SIGMA))));
    }
}

/**
 * @brief Converte a imagem em um grafo, associando cada pixel a um vértice.
 * 
 * Essa função percorre a imagem e cria um grafo onde cada pixel é representado por um vértice. As arestas
 * conectam os pixels vizinhos e têm peso calculado com base na diferença de intensidade entre eles.
 * 
 * @param imagem A imagem a ser convertida em grafo.
 * @param matriz Ponteiro para a matriz que armazenará os vértices do grafo.
 * @return O maior peso (K) encontrado entre os pixels vizinhos.
 */
float imageToGraph(cv::Mat imagem,vector<vector<Vertice *>> *matriz)
{
    float K = -INFINITY;

    // Inicializa a matriz de vértices
    for(int i = 0; i < imagem.size().width; i++)
    {
        vector<Vertice *> initializeMalandra(imagem.size().height);
        matriz->push_back(initializeMalandra);
    }

    matriz->at(0).at(0) = new Vertice(0,0);

    // Percorre cada pixel da imagem e cria vértices no grafo
    for(int i = 0; i < imagem.size().width; i++)
    {
        for(int j = 0; j < imagem.size().height; j++)
        {
            // Conecta os vértices vizinhos
            for(int coluna_offset = -1; coluna_offset <= 1; coluna_offset++)
            {
                for(int linha_offset = -1; linha_offset <= 1; linha_offset++)
                {
                    if(coluna_offset != 0 || linha_offset != 0)
                    {
                        if((i + coluna_offset >=0 && i + coluna_offset < imagem.size().width) && (j + linha_offset >=0 && j + linha_offset < imagem.size().height))
                        {
                            
                            Vertice *aux;

                            if(matriz->at(i + coluna_offset).at(j + linha_offset) == NULL)
                            {
                                aux = new Vertice(i + coluna_offset,j + linha_offset);
                                matriz->at(i + coluna_offset).at(j + linha_offset) = aux;
                            } 
                            else 
                            {
                                aux = matriz->at(i + coluna_offset).at(j + linha_offset);
                            }
                            
                            // Adiciona aresta entre os vértices com peso calculado
                            matriz->at(i).at(j)->addAresta(aux,BorderIndex(imagem.at<uchar>(i,j),imagem.at<uchar>(i+coluna_offset,j+linha_offset)));
                            K = max(K, BorderIndex(imagem.at<uchar>(i,j),imagem.at<uchar>(i+coluna_offset,j+linha_offset)));
                        }
                    }
                }
            }
        }   
    }

    return K;    
}

/**
 * @brief Conecta os nós objeto e fundo com a fonte e o terminal no grafo.
 * 
 * Conecta os pontos de interesse (seeds) de objeto e fundo nos nós da fonte e terminal
 * do grafo com a aresta de peso \( K \), calculado na função `imageToGraph`.
 * 
 * @param matriz A matriz que representa o grafo de vértices.
 * @param K O maior peso das arestas entre vértices vizinhos.
 */
void connectNodesST (vector<vector<Vertice *>> matriz, float K)
{
    // Conecta os pontos de objeto à fonte
    for(int i = 0; i < seedObjeto.size(); i++)
    {
        S->addAresta(matriz.at(seedObjeto.at(i).x).at(seedObjeto.at(i).y), K);
    }

    // Conecta os pontos de fundo ao terminal
    for(int i = 0; i < seedFundo.size(); i++)
    {
        matriz.at(seedFundo.at(i).x).at(seedFundo.at(i).y)->addAresta(T, K);
    }
}

/**
 * @brief Função principal que executa a segmentação.
 * 
 * A função principal realiza os seguintes passos:
 * 1. Cria a imagem marcada com os pontos de objeto e fundo.
 * 2. Converte a imagem em um grafo.
 * 3. Executa o algoritmo de fluxo máximo (Ford-Fulkerson).
 * 4. Realiza o corte mínimo e segmenta a imagem.
 * 
 * @return Código de saída (0 para sucesso).
 */
int main() 
{


    string imagePath = "../Imagens/imagem1.png";  // Caminho da imagem original
    
    // Cria a imagem marcada com pontos de objeto e fundo
    createMarkedImage(imagePath);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    cout << "Criou a imagem marcada" << endl;

    // Carrega a imagem marcada
    imagem = cv::imread("../Imagens/imagemMarcada.png");
    vector<vector<Vertice *>> matriz;

    // Converte a imagem em um grafo
    float K = imageToGraph(imagem, &matriz);
    
    // Conecta os pontos objeto e fundo no grafo
    connectNodesST(matriz, K);

    cout << "Transformou a imagem em um grafo" << endl;

    // Executa o algoritmo de fluxo máximo
    FluxoMaximo(S, T, matriz);

    cout << "O FluxoMaximo terminou" << endl;

    // Realiza o corte mínimo e segmenta a imagem
    corteMinimo(S, T, matriz, imagePath);

    cout << "Segmentação acabou" << endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Tempo de execução: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    
    return 0;
}
