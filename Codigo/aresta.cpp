class Aresta
{
    public:
        int posv1[2];  // Posições do primeiro vértice da aresta (representado por um vetor de duas coordenadas)
        int posv2[2];  // Posições do segundo vértice da aresta (representado por um vetor de duas coordenadas)

        /**
         * @brief Construtor da classe Aresta.
         * 
         * Inicializa a aresta com as posições dos dois vértices (v1 e v2).
         * 
         * @param v1 Um vetor de inteiros representando as coordenadas do primeiro vértice da aresta.
         * @param v2 Um vetor de inteiros representando as coordenadas do segundo vértice da aresta.
         */
        Aresta(int v1[2], int v2[2])
        {
            this->posv1[0] = v1[0];  // Atribui a coordenada x do primeiro vértice
            this->posv1[1] = v1[1];  // Atribui a coordenada y do primeiro vértice

            this->posv2[0] = v2[0];  // Atribui a coordenada x do segundo vértice
            this->posv2[1] = v2[1];  // Atribui a coordenada y do segundo vértice
        }
};
