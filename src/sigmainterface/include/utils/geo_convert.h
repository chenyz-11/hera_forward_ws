#ifndef SIGMA_INTERFACE_GEO_CONVERT_H
#define SIGMA_INTERFACE_GEO_CONVERT_H

/**
 * @file  geo_convert.h
 * @brief 纯函数地理坐标工具：WGS84 经纬高 (LLA) <-> 局部 ENU 直角坐标
 *
 * 本头为 sigma_interface 内部工具（src/），header-only 不对外暴露。
 * 均为无状态的静态函数，输入/输出一律使用「度」和「米」。
 * 协议微度转换（units::kMicroDegToDeg / kDegToMicroDeg）由调用方（WorldState）负责，本层不做。
 */

#include <cmath>

namespace geo {

constexpr double WGS84_A  = 6378137.0;                 // 长半轴 (m)
constexpr double WGS84_F  = 1.0 / 298.257223563;       // 扁率
constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // 第一偏心率平方

struct Vec3 { double x, y, z; };

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

/** @brief 经纬度(度)+高度(m) -> ECEF(m) */
inline Vec3 llaToEcef(double latDeg, double lonDeg, double altM)
{
    double lat = deg2rad(latDeg);
    double lon = deg2rad(lonDeg);
    double sinLat = sin(lat), cosLat = cos(lat);
    double sinLon = sin(lon), cosLon = cos(lon);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
    return {
        (N + altM) * cosLat * cosLon,
        (N + altM) * cosLat * sinLon,
        (N * (1.0 - WGS84_E2) + altM) * sinLat
    };
}

/** @brief ECEF(m) -> 以给定原点 ECEF / 经纬度定义的 ENU(m) */
inline Vec3 ecefToEnu(const Vec3& ori, double homeLatDeg, double homeLonDeg, const Vec3& p)
{
    double lat0 = deg2rad(homeLatDeg);
    double lon0 = deg2rad(homeLonDeg);
    double sinLat = sin(lat0), cosLat = cos(lat0);
    double sinLon = sin(lon0), cosLon = cos(lon0);
    double dx = p.x - ori.x, dy = p.y - ori.y, dz = p.z - ori.z;

    double e = -sinLon * dx + cosLon * dy;
    double n = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    double u =  cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;
    return { e, n, u };
}

/** @brief ENU(m) -> ECEF(m)，原点(home)经纬度定义一致 */
inline Vec3 enuToEcef(const Vec3& ori, double homeLatDeg, double homeLonDeg, const Vec3& enu)
{
    double lat0 = deg2rad(homeLatDeg);
    double lon0 = deg2rad(homeLonDeg);
    double sinLat = sin(lat0), cosLat = cos(lat0);
    double sinLon = sin(lon0), cosLon = cos(lon0);
    double e = enu.x, n = enu.y, u = enu.z;

    double dx = -sinLon * e - sinLat * cosLon * n + cosLat * cosLon * u;
    double dy =  cosLon * e - sinLat * sinLon * n + cosLat * sinLon * u;
    double dz =  cosLat * n + sinLat * u;
    return { ori.x + dx, ori.y + dy, ori.z + dz };
}

/** @brief ECEF(m) -> 经纬度(度)+高度(m)（Bowring 单次求解，无需迭代） */
inline void ecefToLla(const Vec3& p, double* latDeg, double* lonDeg, double* altM)
{
    double b    = WGS84_A * (1.0 - WGS84_F);
    double ep2  = (WGS84_A * WGS84_A - b * b) / (b * b);   // 第二偏心率平方

    double lon = atan2(p.y, p.x);
    double rad = hypot(p.x, p.y);
    double theta = atan2(p.z * WGS84_A, rad * b);
    double sinTh = sin(theta), cosTh = cos(theta);
    double lat = atan2(p.z + ep2 * b * sinTh * sinTh * sinTh,
                       rad - WGS84_E2 * WGS84_A * cosTh * cosTh * cosTh);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin(lat) * sin(lat));
    double h = rad / cos(lat) - N;

    *latDeg = rad2deg(lat);
    *lonDeg = rad2deg(lon);
    *altM   = h;
}

/** @brief 高层便捷接口：给定 home 与目标经纬高(度,m) -> ENU(m) */
inline void llaToEnu(double homeLatDeg, double homeLonDeg, double homeAltM,
                     double latDeg, double lonDeg, double altM,
                     double* e, double* n, double* u)
{
    Vec3 ori = llaToEcef(homeLatDeg, homeLonDeg, homeAltM);
    Vec3 p   = llaToEcef(latDeg, lonDeg, altM);
    Vec3 enu = ecefToEnu(ori, homeLatDeg, homeLonDeg, p);
    *e = enu.x; *n = enu.y; *u = enu.z;
}

/** @brief 高层便捷接口：ENU(m) -> 经纬度(度)+高度(m) */
inline void enuToLla(double homeLatDeg, double homeLonDeg, double homeAltM,
                     double e, double n, double u,
                     double* latDeg, double* lonDeg, double* altM)
{
    Vec3 ori = llaToEcef(homeLatDeg, homeLonDeg, homeAltM);
    Vec3 p   = enuToEcef(ori, homeLatDeg, homeLonDeg, { e, n, u });
    ecefToLla(p, latDeg, lonDeg, altM);
}

} // namespace geo

#endif // GEO_CONVERT_H