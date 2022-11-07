
#include <SPI.h>    
#include <SD.h>     

#define SSpin 10    

File archivo;    

void setup() {
  Serial.begin(9600);      
  Serial.println("Inicializando tarjeta ...");  
  if (!SD.begin(SSpin)) {     
    Serial.println("fallo en inicializacion !");
    return;        
  }
  
  Serial.println("inicializacion correcta");  
  archivo = SD.open("prueba.txt", FILE_WRITE); 

  if (archivo) {
    archivo.println("Probando 1, 2, 3"); 
    Serial.println("Escribiendo en archivo prueba.txt..."); 
    archivo.close();       
    Serial.println("escritura correcta"); 
  } else {
    Serial.println("error en apertura de prueba.txt"); 
  }

  archivo = SD.open("prueba.txt"); 
  if (archivo) {
    Serial.println("Contenido de prueba.txt:"); 
    while (archivo.available()) {  
      Serial.write(archivo.read());   
    }
    archivo.close();      
  } else {
    Serial.println("error en la apertura de prueba.txt");
  }
}

void loop() { 
  
}