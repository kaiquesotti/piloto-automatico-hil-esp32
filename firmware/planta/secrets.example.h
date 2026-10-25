#pragma once

// -----------------------------------------------------------------------------
//  secrets.example.h -- modelo de credenciais da rede Wi-Fi
// -----------------------------------------------------------------------------
//  Copie este arquivo para secrets.h e preencha com os dados da sua rede:
//
//      cp firmware/planta/secrets.example.h firmware/planta/secrets.h
//
//  secrets.h esta listado no .gitignore e NAO deve ser versionado.
//  A planta entra na rede como estacao (STA); o computador de supervisao
//  precisa estar na mesma rede para abrir a pagina web.
// -----------------------------------------------------------------------------

#define WIFI_SSID "NOME_DA_REDE"
#define WIFI_PASSWORD "SENHA_DA_REDE"
