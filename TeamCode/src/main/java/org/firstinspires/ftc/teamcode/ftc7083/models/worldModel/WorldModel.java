package org.firstinspires.ftc.teamcode.models.worldModel;


import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WorldModel {
    private final Map<String, WorldObject> worldModelObjects = new HashMap<>();


    public void setValue(WorldObject value) {
        worldModelObjects.put(value.key, value);
    }

    public WorldObject getValue(String key) {
        return worldModelObjects.get(key);
    }

    public /* WorldObject[] */ void getValues(List<String> keys) {
/*
        var x = worldModelObjects.entrySet()
                .stream()
                .filter((entry) -> keys.contains(entry.getKey()))
                .map(Map.Entry::getValue)
                .collect(Collectors.toList());

        System.out.println("World model values" + x);
        */
        // return
        // worldModelObjects.entrySet().stream().filter((entry)->keys.equals(entry.getKey()));
    }
}

