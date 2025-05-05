#include <png.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <memory>
#include "pixel.cpp"
#include "aresta.cpp"
#include "componente.cpp"

#include <chrono>

using namespace std;

// Constante K utilizada em cálculos de pesos
const float K = 420;

// Váriáveis globais que armazenam a altura e a largura da imagem escolhida
int altura;
int largura;

/**
 * @brief Cria um arquivo de imagem no formato PPM (Portable Pixel Map) a partir de uma matriz de intensidades.
 * 
 * O arquivo gerado contém informações de largura, altura e intensidade máxima de cor (255).
 *
 * @param mat Matriz 2D de inteiros representando as intensidades dos pixels da imagem.
 */
void createImageRGB(vector<vector<int>> mat)
{
    // Abre o arquivo para escrita
    ofstream imagem("../Imagens/imageGray.ppm");

    // Verifica se o arquivo foi aberto corretamente
    if (!imagem) 
    {
        std::cerr << "Erro ao abrir o arquivo para escrita." << "\n";
    }
    else
    {
        // Escreve cabeçalho do arquivo PPM com largura, altura e intensidade máxima de cor
        imagem << "P3\n" << largura << " " << altura << "\n" << "255\n";

        // Escreve os dados dos pixels da imagem
        for(int linha = 0; linha < altura; linha++)
        {
            for(int coluna = 0; coluna < largura; coluna++)
            {
                // Escreve os valores de intensidade normalizados e formatados para PPM
                imagem << mat[linha][coluna] << " " << mat[linha][coluna] << " " << mat[linha][coluna] <<  " ";
            }
            
            // Finaliza a linha de pixels
            imagem << "\n";
        }

        // Fecha o arquivo após a escrita completa
        imagem.close();

        // Exibe mensagem de sucesso
        cout << "Imagem criada com sucesso." << "\n";
    }
}

/**
 * @brief Inicializa a imagem a partir de um arquivo PNG, convertendo-a para uma matriz de componentes.
 * 
 * Lê uma imagem PNG e converte seus valores de pixel para uma matriz 2D de componentes e arestas.
 * Também cria uma imagem em escala de cinza para visualização.
 *
 * @param matrizPixel Matrizes de componentes onde serão armazenados os componentes gerados da imagem.
 * @return Lista de arestas de diferença de intensidade entre os pixels.
 */
vector<vector<Aresta *>> init_imagem(vector<vector<Componente *>> *matrizPixel)
{

    FILE *fraw = fopen("../Imagens/PacManGaussian.png","rb");  // Abre a imagem PNG
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING,NULL,NULL,NULL);

    png_infop png_info = png_create_info_struct(png);

    png_init_io(png,fraw);

    png_read_info(png,png_info);

    altura = png_get_image_height(png,png_info);  // Obtém a altura da imagem
    largura = png_get_image_width(png,png_info);  // Obtém a largura da imagem

    png_set_strip_alpha(png);
    png_set_rgb_to_gray(png, 1, 0.45, 0.45);  // Converte para escala de cinza

    png_bytep rgb[altura];
    
    for (int i = 0; i < altura; i++)
    {
        rgb[i] = (png_byte *)malloc(png_get_rowbytes(png,png_info));
    }

    png_read_image(png,rgb);  // Lê os dados da imagem PNG

    vector<vector<Aresta *>> listaDiferencaArestas(256);
    vector<vector<int>> TesteImagem;

    // Identificador do Componente
    int label = 0;

    for (int i = 0; i < altura; i++)
    {
        vector<int> testes;
        vector<Componente *> listaPixels;

        for (int j = 0 ; j < largura; j++)
        {
            testes.push_back(rgb[i][j]);
            int v1[2] = {i,j};
            Pixel *p = new Pixel(i,j);
            listaPixels.push_back(new Componente(p, label++));

            for(int linha = 0; linha <= 1; linha++)
            {
                for(int coluna = -linha; coluna <= 1; coluna++)
                {
                    if(linha != 0 || coluna!= 0)
                    {
                        if((i + linha >= 0 && i + linha < altura) && (j + coluna >= 0 && j + coluna < largura))
                        {
                            int v2[2] = {i+linha,j+coluna};
                            // Adiciona aresta com a diferença de intensidade entre os pixels adjacentes
                            listaDiferencaArestas.at(abs((int)rgb[i][j]-(int)rgb[i+linha][j+coluna])).push_back(new Aresta(v1,v2));
                        }
                    }
                }
            }
        }

        matrizPixel->push_back(listaPixels);
        TesteImagem.push_back(testes);
    }

    // Cria uma imagem PPM para visualização
    createImageRGB(TesteImagem);

    // Libera a memória utilizada pela leitura da imagem
    for (int i = 0; i < altura; i++)
    {
        free(rgb[i]);
    }

    png_destroy_read_struct(&png,&png_info,NULL);
    fclose(fraw);  // Fecha o arquivo PNG

    return listaDiferencaArestas;
}

/**
 * @brief Calcula o mínimo entre os maiores pesos de dois componentes.
 * 
 * O cálculo é baseado na fórmula fornecida, considerando o maior peso de cada componente e sua cardinalidade.
 * 
 * @param c1 Componente 1.
 * @param c2 Componente 2.
 * @return O valor mínimo calculado entre os dois componentes.
 */
float MInt(Componente c1, Componente c2)
{
    return min(c1.getMaiorPeso() + (K / c1.getCardinalidade()), c2.getMaiorPeso() + (K / c2.getCardinalidade()));
}

/**
 * @brief Cria um arquivo de imagem no formato PPM a partir de uma matriz de componentes.
 * 
 * Cada componente é atribuído uma cor aleatória para visualização. O arquivo gerado é um mapa de pixels no formato PPM.
 *
 * @param mat Matriz de componentes que contém os componentes de pixels da imagem.
 */
void createImageRGB(vector<vector<Componente *>> mat)
{
    // Abre o arquivo para escrita
    ofstream imagem("../Imagens/image.ppm");

    // Verifica se o arquivo foi aberto corretamente
    if (!imagem) 
    {
        std::cerr << "Erro ao abrir o arquivo para escrita." << "\n";
    }
    else
    {
        // Escreve cabeçalho do arquivo PPM com largura, altura e intensidade máxima de cor
        imagem << "P3\n" << largura << " " << altura << "\n" << "255\n";

        // Escreve os dados dos pixels da imagem
        for(int linha = 0; linha < altura; linha++)
        {
            for(int coluna = 0; coluna < largura; coluna++)
            {
                Componente *aux = mat.at(linha).at(coluna);
                srand(aux->getlabel()+1);
                // Atribui uma cor aleatória a cada componente
                imagem << rand() % 255 << " " << rand() % 255 << " " << rand() % 255 <<  " ";
            }
            
            // Finaliza a linha de pixels
            imagem << "\n";
        }

        // Fecha o arquivo após a escrita completa
        imagem.close();

        // Exibe mensagem de sucesso
        cout << "Imagem criada com sucesso." << "\n";
    }
}

/**
 * @brief Mescla dois componentes em um único componente.
 * 
 * Os pixels de um componente são movidos para o outro e o maior peso é atualizado.
 * 
 * @param c1 Componente 1.
 * @param c2 Componente 2.
 * @param matrizComponentes Matrizes de componentes para atualizar a referência de componentes.
 * @param peso O peso da aresta que conecta os dois componentes.
 */
void mergeComponentes(Componente *c1, Componente *c2, vector<vector<Componente *>> *matrizComponentes, float peso)
{
    if(c1->getCardinalidade() <= c2->getCardinalidade())
    {
        // Move os pixels de c1 para c2
        while(!c1->getListaPixels()->empty()){
            Pixel *p = c1->getListaPixels()->back();
            c2->getListaPixels()->push_back(p);
            matrizComponentes->at(p->getPos()[0]).at(p->getPos()[1]) = c2;
            c1->getListaPixels()->erase(c1->getListaPixels()->end());
        }
        c2->setMaiorPeso(max(peso,max(c2->getMaiorPeso(), c1->getMaiorPeso())));
    } 
    else 
    {
        // Move os pixels de c2 para c1
        while(!c2->getListaPixels()->empty())
        {
            Pixel *p = c2->getListaPixels()->back();
            c1->getListaPixels()->push_back(p);
            matrizComponentes->at(p->getPos()[0]).at(p->getPos()[1]) = c1;
            c2->getListaPixels()->erase(c2->getListaPixels()->end());
        }

        c1->setMaiorPeso(max(peso,max(c2->getMaiorPeso(), c1->getMaiorPeso())));
    }
}

int main()
{

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    vector<vector<Componente *>> matrizComponentes;
    // Inicializa a imagem e obtém a lista de arestas
    vector<vector<Aresta *>> listaArestas = init_imagem(&matrizComponentes);

    cout << altura << " " << largura << endl;

    // Mescla componentes baseando-se nas arestas e seus pesos
    for(int i = 0; i < listaArestas.size(); i++)
    {
        for(int j = 0; j < listaArestas.at(i).size(); j++)
        {
            if(matrizComponentes.at(listaArestas.at(i).at(j)->posv1[0]).at(listaArestas.at(i).at(j)->posv1[1])->getlabel() != matrizComponentes.at(listaArestas.at(i).at(j)->posv2[0]).at(listaArestas.at(i).at(j)->posv2[1])->getlabel())
            {
                if(i <= MInt(*matrizComponentes.at(listaArestas.at(i).at(j)->posv1[0]).at(listaArestas.at(i).at(j)->posv1[1]) , *matrizComponentes.at(listaArestas.at(i).at(j)->posv2[0]).at(listaArestas.at(i).at(j)->posv2[1])))
                {
                
                    // Mescla os componentes se a condição for atendida
                    mergeComponentes(matrizComponentes.at(listaArestas.at(i).at(j)->posv1[0]).at(listaArestas.at(i).at(j)->posv1[1]), matrizComponentes.at(listaArestas.at(i).at(j)->posv2[0]).at(listaArestas.at(i).at(j)->posv2[1]), &matrizComponentes, i);
                }
            }
        }
    }

    // Cria uma imagem de resultado com os componentes mesclados
    createImageRGB(matrizComponentes);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Tempo de execução: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

    return 0;
}
