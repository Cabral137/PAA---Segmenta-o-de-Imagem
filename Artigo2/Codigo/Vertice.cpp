#include <vector> // Para armazenar as arestas

class Vertice
{
    private:
        std::vector<Vertice *> aresta;     // Lista de vértices conectados por arestas
        std::vector<float> pesoAresta;     // Lista de pesos das arestas

        int x;  // Coordenada X do vértice
        int y;  // Coordenada Y do vértice
    
    public:
        
        /**
         * @brief Construtor da classe Vertice.
         * 
         * Este construtor cria um vértice com as coordenadas especificadas.
         * 
         * @param x Coordenada X do vértice.
         * @param y Coordenada Y do vértice.
         */
        Vertice(int x, int y)
        {
            this->x = x;
            this->y = y;
        }

        /**
         * @brief Construtor padrão da classe Vertice.
         * 
         * Este construtor cria um vértice com as coordenadas (-1, -1), o que pode
         * indicar um vértice "nulo" ou não inicializado.
         */
        Vertice()
        {
            this->x = -1;
            this->y = -1;
        }

        /**
         * @brief Adiciona uma aresta ao vértice.
         * 
         * Essa função adiciona uma aresta entre o vértice atual e o vértice fornecido,
         * além de atribuir o peso da aresta.
         * 
         * @param v Ponteiro para o vértice vizinho conectado por esta aresta.
         * @param peso O peso da aresta.
         */
        void addAresta(Vertice *v, float peso)
        {
            this->aresta.push_back(v);       // Adiciona o vértice vizinho à lista de arestas
            this->pesoAresta.push_back(peso); // Adiciona o peso da aresta à lista de pesos
        }

        /**
         * @brief Obtém a coordenada X do vértice.
         * 
         * @return A coordenada X do vértice.
         */
        int getX()
        {
            return this->x;
        }

        /**
         * @brief Obtém a coordenada Y do vértice.
         * 
         * @return A coordenada Y do vértice.
         */
        int getY()
        {
            return this->y;
        }

        /**
         * @brief Obtém a lista de pesos das arestas do vértice.
         * 
         * @return Um vetor contendo os pesos das arestas conectadas a este vértice.
         */
        std::vector<float> getListaPesos()
        {
            return this->pesoAresta;
        }

        /**
         * @brief Atualiza o peso de uma aresta específica.
         * 
         * A função permite modificar o peso de uma aresta existente, dado o índice da aresta
         * na lista de arestas e pesos.
         * 
         * @param index O índice da aresta a ser atualizada.
         * @param peso O novo peso da aresta.
         */
        void setPesoArray(int index, float peso)
        {
            this->pesoAresta.at(index) = peso; // Atualiza o peso da aresta no índice especificado
        }

        /**
         * @brief Obtém a lista de vértices vizinhos conectados ao vértice atual.
         * 
         * @return Um vetor contendo ponteiros para os vértices vizinhos conectados por arestas.
         */
        std::vector<Vertice *> getListaVertices()
        {
            return this->aresta;
        }
};
