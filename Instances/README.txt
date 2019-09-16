As instâncias fornecidas se dividem em dois tipos que são aquelas onde a distância entre os pontos é euclidiana (EUC_2D) e as que a distância é pseudo euclidiana (ATT). 
Por isso, para cada tipo de instância, o cálculo da distância entre os pontos é feito de forma diferente.

1- Distância Euclidiana (EUC_2D)

Sejam x[i] e y[i] as coordenadas de um ponto i. 
Para as arestas do tipo EUC_2D (distância euclidiana) a 
distância entre dois pontos i e j é computado como segue:

xd= x[i]-x[j];
yd= y[i]-y[j];
dij= nint( sqrt( xd*xd + yd*yd));


2- Distância Euclidiana (ATT)

Sejam x[i] e y[i] as coordenadas de um ponto i. 
Para as arestas do tipo ATT (distância pseudo euclidiana) a 
distância entre dois pontos i e j é computado como segue:

xd= x[i]-x[j];
yd= y[i]-y[j];
rij= sqrt( (xd*xd + yd*yd)/10.0 );
tij= nint( rij );

if(tij<rij)
 dij= tij+1;
else
   dij= tij;
