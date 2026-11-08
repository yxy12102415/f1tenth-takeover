#include "perception.hpp"
#include "joint_control.hpp"
#include <iostream>
#include <memory>
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  try {
    frenet::Assets assets(argv[1]);
    std::unique_ptr<frenet::Planner> planner(new frenet::Planner(assets));
    std::unique_ptr<frenet::Tracker> tracker;
    std::unique_ptr<frenet::JointControl> joint;
    frenet::Perception perception(assets);
    std::string line;
    while (std::getline(std::cin, line)) {
      Json::Value out;
      try {
        Json::CharReaderBuilder builder;
        Json::Value j;
        std::string errors;
        std::istringstream input(line);
        if (!Json::parseFromStream(builder, input, &j, &errors))
          throw std::runtime_error("json_invalid");
        std::string op = j.get("op", "plan").asString();
        if (op == "plan") {
          if (j.get("reset", false).asBool())
            planner.reset(new frenet::Planner(assets));
          if (j.isMember("progress"))
            planner->progress = j["progress"].asInt();
          frenet::Pose pose{j["pose"][0].asDouble(), j["pose"][1].asDouble(),
                            j["pose"][2].asDouble()};
          std::vector<frenet::Point> obs;
          for (auto p : j["obstacles"])
            obs.emplace_back(p[0].asDouble(), p[1].asDouble());
          double begin = frenet::mono();
          auto c = planner->plan(pose, obs, j.get("horizon", 3.2).asDouble());
          out = frenet::candidate_json(c);
          out["compute_ms"] = (frenet::mono() - begin) * 1000;
          out["mode"] = planner->mode;
          out["switch_reason"] = planner->switch_reason;
          out["switches"] = planner->switches;
          out["progress"] = planner->progress;
          out["center_path_reused"] = planner->reused;
          out["candidates"] = Json::Value(Json::arrayValue);
          if (j.get("candidates", false).asBool())
            for (auto candidate : planner->candidates)
              out["candidates"].append(frenet::candidate_json(candidate));
        } else if (op == "joint") {
          std::string type=j.get("type","dynamic_mpc").asString();
          if(!tracker || tracker->type!=type || j.get("reset",false).asBool()) {
            tracker.reset(new frenet::Tracker(type));joint.reset(new frenet::JointControl(type=="dynamic_mpc"));
          }
          // Non-actuating probe only: bounded experiments for PID selection.
          if(type=="pid" && j["pid_tuning"].isArray() && j["pid_tuning"].size()==3) {
            tracker->pid_kp=j["pid_tuning"][0].asDouble();
            tracker->pid_preview_gain=j["pid_tuning"][1].asDouble();
            tracker->pid_kd=j["pid_tuning"][2].asDouble();
          }
          if(type=="pid" && j.isMember("pid_prediction_s")) tracker->pid_prediction_s=j["pid_prediction_s"].asDouble();
          std::vector<frenet::Point> obs;for(auto o:j["obstacles"])obs.emplace_back(o[0].asDouble(),o[1].asDouble());
          frenet::Pose pose{j["pose"][0].asDouble(),j["pose"][1].asDouble(),j["pose"][2].asDouble()};
          auto c=joint->compute(assets,frenet::path_read(j["path"]),pose,j.get("speed",0.).asDouble(),j.get("lateral",0.).asDouble(),j.get("yaw_rate",0.).asDouble(),j["remaining"].asDouble(),j.get("cap",2.0).asDouble(),j.get("dt",.05).asDouble(),*tracker,obs);
          out["speed"]=c.speed;out["angle"]=c.angle;out["diag"]=joint->diag;
        } else if (op == "track") {
          std::string type = j.get("type", "dynamic_mpc").asString();
          if (!tracker || tracker->type != type ||
              j.get("reset", false).asBool())
            tracker.reset(new frenet::Tracker(type));
          frenet::Pose pose{j["pose"][0].asDouble(), j["pose"][1].asDouble(),
                            j["pose"][2].asDouble()};
          out["angle"] = tracker->compute(
              frenet::path_read(j["path"]), pose, j.get("dt", .05).asDouble(),
              j.get("speed", .75).asDouble(), j.get("lateral", 0.).asDouble(),
              j.get("yaw_rate", 0.).asDouble());
          out["diag"] = tracker->diag;
        } else if (op == "perception") {
          if (j.get("reset", false).asBool()) {
            perception.tracks.clear();
            perception.next_id = 1;
            perception.last_stamp = -frenet::INF;
          }
          std::vector<frenet::Point> p;
          std::vector<int> beams;
          for (auto v : j["points"])
            p.emplace_back(v[0].asDouble(), v[1].asDouble());
          for (auto v : j["beams"])
            beams.push_back(v.asInt());
          auto clusters = perception.cluster(p, beams);
          auto points = perception.update(clusters, j["stamp"].asDouble());
          out = perception.diagnostic();
          out["points"] = Json::Value(Json::arrayValue);
          for (auto p : points) {
            Json::Value v(Json::arrayValue);
            v.append(p.x());
            v.append(p.y());
            out["points"].append(v);
          }
        } else
          throw std::runtime_error("unknown_operation");
        out["ok"] = true;
      } catch (const std::exception &e) {
        out["ok"] = false;
        out["error"] = e.what();
        if (joint) out["rejected_diagnostics"] = joint->diag;
      }
      std::cout << frenet::dump(out) << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
