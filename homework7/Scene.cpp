//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Vector3f L_dir = {0,0,0}, L_indir{0,0,0};

    Intersection intersection = Scene::intersect(ray);
    if(!intersection.happened)
        return {};
    if(intersection.m->hasEmission()){
        //交点是光源
        return intersection.m->getEmission();
    }

    Intersection lightPos;
    float lightPdf = 0.0f;

    sampleLight(lightPos, lightPdf);

    Vector3f collisionLight = lightPos.coords - intersection.coords;//反射光
    float dis = dotProduct(collisionLight, collisionLight);
    Vector3f collisionLightDir = collisionLight.normalized();

    Ray lightToObject(intersection.coords,collisionLightDir);

    Intersection lightToAny = Scene::intersect(lightToObject);

    auto f_r = intersection.m->eval(ray.direction, collisionLightDir, intersection.normal);

    if(lightToAny.distance-collisionLight.norm()>-0.005){
        L_dir = lightPos.emit * f_r * dotProduct(collisionLightDir, intersection.normal)
            * dotProduct(-collisionLightDir, lightPos.normal)/dis/lightPdf;
    }

    if(get_random_float()>RussianRoulette)
        return L_dir;

    Vector3f wi = intersection.m->sample(ray.direction,intersection.normal).normalized();
    Vector3f w0 = ray.direction;

    Ray objToObj(intersection.coords, wi);
    Intersection isLight = Scene::intersect(objToObj);

    if(isLight.happened&&!isLight.m->hasEmission()){
        float pdf = intersection.m->pdf(w0,wi,intersection.normal);
        f_r = intersection.m->eval(w0,wi,intersection.normal);
        L_indir = castRay(objToObj, depth+1) * f_r * dotProduct(wi,intersection.normal)/pdf/RussianRoulette;
    }

    return L_dir+L_indir;

}

