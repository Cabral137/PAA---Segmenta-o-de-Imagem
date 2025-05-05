#include <vector>  // Para usar o tipo vetor

class Componente
{
    private:
        std::vector <Pixel*> listaPixels;  // Lista de ponteiros para os pixels que fazem parte deste componente
        float maiorPeso;  // Maior peso associado ao componente
        int label;  // Label único para identificar o componente

    public:
        /**
         * @brief Construtor da classe Componente.
         * 
         * Inicializa um componente com um pixel e atribui um label único.
         *
         * @param Pixel Ponteiro para um pixel que faz parte deste componente.
         * @param l Label único para o componente.
         */
        Componente(Pixel *Pixel, int l)
        {
            this->listaPixels.push_back(Pixel);  // Adiciona o pixel à lista de pixels do componente
            maiorPeso = 0;  // Inicializa o maior peso com zero
            label = l;  // Atribui o label ao componente
        }

        /**
         * @brief Retorna o maior peso associado ao componente.
         * 
         * @return O maior peso do componente.
         */
        float getMaiorPeso()
        {
            return this->maiorPeso;
        }

        /**
         * @brief Retorna a cardinalidade (tamanho) do componente.
         * 
         * A cardinalidade é o número de pixels que pertencem ao componente.
         * 
         * @return O número de pixels do componente.
         */
        int getCardinalidade()
        {
            return this->listaPixels.size();
        }

        /**
         * @brief Retorna o label único do componente.
         * 
         * @return O label do componente.
         */
        int getlabel()
        {
            return this->label;
        }

        /**
         * @brief Define o maior peso do componente.
         * 
         * A função define o maior peso associado ao componente.
         * 
         * @param p O novo maior peso a ser atribuído ao componente.
         */
        void setMaiorPeso(float p)
        {
            this->maiorPeso = p;
        }

        /**
         * @brief Retorna a lista de pixels que pertencem ao componente.
         * 
         * A função retorna um ponteiro para a lista de pixels.
         * 
         * @return Ponteiro para a lista de pixels.
         */
        std::vector<Pixel *> *getListaPixels()
        {
            return &this->listaPixels;  // Retorna o ponteiro para a lista de pixels
        }
};
