#ifndef PLCOPENMCKERNELOSAL
#define PLCOPENMCKERNELOSAL
#include"ThreadManage.h"
// #include"TimerManager.h"

#ifdef USELOGLIB
#include<spdlog/spdlog.h>
#define EC_TRACE(...) 				spdlog::trace(__VA_ARGS__)
#define EC_INFO(...) 				spdlog::info(__VA_ARGS__)
#define EC_DEBUG(...) 				spdlog::debug(__VA_ARGS__)
#define EC_ERROR(...) 				spdlog::error(__VA_ARGS__)
#define EC_WARN(...) 				spdlog::warn(__VA_ARGS__)
#else
#define EC_TRACE(...) 				
#define EC_INFO(...) 				
#define EC_DEBUG(...) 				
#define EC_ERROR(...) 				
#define EC_WARN(...) 				
#endif
#endif