/*!
 * @file OpenFIREversion.h
 * @brief OpenFIREversion.h
 * @n CPP OpenFIREversion.h
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2026
 */

#ifndef OPENFIRE_VERSION_H
#define OPENFIRE_VERSION_H

// Versione attuale
#define OPENFIRE_VERSION 6.2
///////#define OPENFIRE_CODENAME "Dawn Sigma rc2"
//#define GIT_HASH
#define OPENFIRE_VERSION_MAJOR 6
#define OPENFIRE_VERSION_MINOR 2
#define OPENFIRE_VERSION_PATCH 0
#define OPENFIRE_VERSION_TYPE "Beta1"  // Per indicare pre-release (alpha, beta, RC(Release Candidate), stable, dev)

// Stringa di versione attuale
#define OPENFIRE_VERSION_STRING (String(OPENFIRE_VERSION_MAJOR) + "." + String(OPENFIRE_VERSION_MINOR) + "." + String(OPENFIRE_VERSION_PATCH)+ "-" + OPENFIRE_VERSION_TYPE)
//#define OPENFIRE_VERSION_STRING TU_STRING(OPENFIRE_VERSION_MAJOR) "." TU_STRING(OPENFIRE_VERSION_MINOR) "." TU_STRING(OPENFIRE_VERSION_PATCH)

// Numero combinato della versione attuale
#define OPENFIRE_VERSION_NUMBER (OPENFIRE_VERSION_MAJOR * 10000 + OPENFIRE_VERSION_MINOR * 100 + OPENFIRE_VERSION_PATCH)

// =============== ultima versione compatibile =================================

// Ultima versione precedente compatibile
#define OPENFIRE_COMPATIBLE_MAJOR 4
#define OPENFIRE_COMPATIBLE_MINOR 3
#define OPENFIRE_COMPATIBLE_PATCH 2

// Stringa di versione compatibile precedente
#define OPENFIRE_COMPATIBLE_STRING (String(OPENFIRE_COMPATIBLE_MAJOR) + "." + String(OPENFIRE_COMPATIBLE_MINOR) + "." + String(OPENFIRE_COMPATIBLE_PATCH))
//#define OPENFIRE_COMPATIBLE_STRING TU_STRING(OPENFIRE_COMPATIBLE_MAJOR) "." TU_STRING(OPENFIRE_COMPATIBLE_MINOR) "." TU_STRING(OPENFIRE_COMPATIBLE_PATCH)

// Numero combinato della versione precedente compatibile
#define OPENFIRE_COMPATIBLE_NUMBER (OPENFIRE_COMPATIBLE_MAJOR * 10000 + OPENFIRE_COMPATIBLE_MINOR * 100 + OPENFIRE_COMPATIBLE_PATCH)

// ---------------------------------------------------------------------------
// App 配对渠道标记（方案 A）：在 platformio 中定义 OPENFIRE_APP_FORK_BUILD 后，
// Dock 串口首段版本号末尾会追加 "+<tag>"，与官方固件区分且 semver 仍为 6.x。
// 可选在编译参数中覆盖：-D OPENFIRE_APP_FORK_TAG_STR=\"mytag\"
// ---------------------------------------------------------------------------
#ifdef OPENFIRE_APP_FORK_BUILD
#ifndef OPENFIRE_APP_FORK_TAG_STR
#define OPENFIRE_APP_FORK_TAG_STR "cnwei"
#endif
#endif

#endif  // OPENFIRE_VERSION_H
