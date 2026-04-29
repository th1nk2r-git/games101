//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const {
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const {
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const {

    Vector3f L_dir = 0, L_indir = 0;

    Intersection p_inter = intersect(ray);

    if (p_inter.happened == false) {
        return Vector3f(0, 0, 0);
    }

    if (p_inter.m->hasEmission() == true) {
        return p_inter.m->getEmission();
    }

    auto shading_point = p_inter.coords;

    Intersection light_inter;
    float pdf_light;
    sampleLight(light_inter, pdf_light);

    // calculate L_dir
    auto emit_point = light_inter.coords;

    auto ws_dir = (emit_point - shading_point).normalized();

    auto test_inter = intersect(Ray(shading_point, ws_dir));
    float dis_to_light = (emit_point - shading_point).norm();
    if (test_inter.distance >= dis_to_light - 0.0001) {
        auto L_i = light_inter.emit;
        auto f_r = p_inter.m->eval(ray.direction, ws_dir, p_inter.normal);
        auto cos1 = std::max(0.f, dotProduct(p_inter.normal.normalized(), ws_dir));
        auto cos2 = std::max(0.f, dotProduct(light_inter.normal.normalized(), -ws_dir));
        auto d_factor = pow((emit_point - shading_point).norm(), 2);
        L_dir = L_i * f_r * cos1 * cos2 / d_factor / pdf_light;
    }

    // calculate L_indir
    if (get_random_float() <= RussianRoulette) {
        auto wi = -p_inter.m->sample(ray.direction, p_inter.normal.normalized()).normalized();
        auto wo = -ray.direction;
        auto N = p_inter.normal.normalized();
        Ray traced_ray(shading_point, -wi);
        auto inter = intersect(traced_ray);
        if (inter.happened && inter.m->hasEmission() == false && depth < maxDepth) {
            auto L = castRay(traced_ray, depth + 1);
            auto f_r = p_inter.m->eval(-wo, -wi, N);
            auto cos_theta = dotProduct(-wi, N);
            auto pdf = p_inter.m->pdf(-wo, -wi, N);
            
            L_indir = L * f_r * cos_theta / pdf / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}
