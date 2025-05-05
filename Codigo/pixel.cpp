class Pixel
{
    private:
        int pos[2];  // Array que armazena as coordenadas (x, y) do pixel

    public:
        /**
         * @brief Construtor da classe Pixel.
         * 
         * Inicializa um pixel com as coordenadas fornecidas (x, y).
         * 
         * @param x Coordenada x do pixel.
         * @param y Coordenada y do pixel.
         */
        Pixel(int x, int y)
        {
            this->pos[0] = x;  // Atribui a coordenada x do pixel
            this->pos[1] = y;  // Atribui a coordenada y do pixel
        }

        /**
         * @brief Retorna as coordenadas do pixel.
         * 
         * Retorna um ponteiro para o array que contém as coordenadas (x, y) do pixel.
         * 
         * @return Ponteiro para o array de coordenadas [x, y] do pixel.
         */
        int* getPos()
        {
            return this->pos;  // Retorna o array de coordenadas do pixel
        }
};
