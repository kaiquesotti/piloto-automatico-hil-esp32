#pragma once

// =============================================================================
//  secrets.example.h — MODELO de credenciais Wi-Fi (arquivo de exemplo)
// =============================================================================
//
//  O sketch "planta.ino" precisa de um arquivo chamado "secrets.h" com o nome
//  e a senha da sua rede Wi-Fi. Por segurança, esse arquivo NÃO existe no
//  repositório: ele fica apenas na sua máquina.
//
//  COMO USAR (3 passos)
//  --------------------
//  1. Copie este arquivo para "secrets.h", na mesma pasta.
//
//       Windows (Prompt de Comando):
//         copy secrets.example.h secrets.h
//
//       Linux / macOS:
//         cp secrets.example.h secrets.h
//
//     Pelo Explorador de Arquivos: selecione "secrets.example.h", copie e
//     cole na mesma pasta; depois renomeie a cópia para "secrets.h".
//
//  2. Abra o novo "secrets.h" e troque os textos entre aspas pelos dados da
//     sua rede. Mantenha as aspas. Exemplo:
//
//         #define WIFI_SSID "MinhaCasa"
//         #define WIFI_PASSWORD "minha_senha"
//
//  3. Compile e grave "planta.ino". O "secrets.h" é ignorado pelo Git e não
//     será enviado ao GitHub.
//
//  IMPORTANTE
//  ----------
//  - Não renomeie "secrets.example.h": ele é o modelo público.
//  - Se "secrets.h" não existir, a compilação para com a mensagem
//    "secrets.h: No such file or directory". Isso é esperado: volte ao passo 1.
//  - A placa entra como estação (STA) na rede informada; o computador de
//    supervisão precisa estar na mesma rede.
//  - O ESP32-WROOM-32 opera em 2,4 GHz; redes de 5 GHz não são suportadas.
// =============================================================================

#define WIFI_SSID "NOME_DA_REDE"
#define WIFI_PASSWORD "SENHA_DA_REDE"
