package org.firstinspires.ftc.teamcode.TurretCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;

/*
 * TagClusters - figure out WHICH group of AprilTags the camera is looking at.
 *
 * A "cluster" is a named group of AprilTag IDs, for example "blue 1" = tags 1, 2, 3.
 * The HuskyLens only reports one tag ID per block, so we do the grouping ourselves:
 *   1. Count how many tags from each cluster are visible right now.
 *   2. The cluster with the most visible tags wins.
 *   3. If too few of its tags are visible, we say "no cluster" instead of guessing.
 *
 * How to use it (after camera.update() has run this loop):
 *   TagClusters.Result r = TagClusters.identify(camera.getBlocks());
 *   if (r.name != null) { ... r.name is "blue 1", "blue 2", "red 1" or "red 2" ... }
 *
 * SETUP: Put the real tag IDs in CLUSTERS below. Every tag ID must belong to only ONE cluster.
 */
public class TagClusters {

    // TUNE: the tag IDs in each cluster. These numbers are placeholders - change them to the
    // IDs you print on the tags. Each cluster can have as many tags as you like.
    public static final Cluster[] CLUSTERS = {
            new Cluster("blue 1", 1, 2, 3),
            new Cluster("blue 2", 4, 5, 6),
            new Cluster("red 1",  7, 8, 9),
            new Cluster("red 2",  10, 11, 12),
    };

    // TUNE: how many tags from a cluster must be visible before we call it "found".
    // 1 = any single tag is enough. Raise it to 2 if you get false matches.
    public static int MIN_TAGS_TO_MATCH = 1;

    /** A named group of tag IDs. */
    public static class Cluster {
        public final String name;
        public final int[] tagIds;

        public Cluster(String name, int... tagIds) {
            this.name = name;
            this.tagIds = tagIds;
        }
    }

    /** What the camera sees this loop. name is null when no cluster was found. */
    public static class Result {
        public final String name;       // "blue 1", "red 2", ... or null
        public final int tagsFound;     // how many tags of that cluster are visible
        public final int tagsInCluster; // how many tags that cluster has in total

        Result(String name, int tagsFound, int tagsInCluster) {
            this.name = name;
            this.tagsFound = tagsFound;
            this.tagsInCluster = tagsInCluster;
        }

        /** Like the SDK's "Percent tags found": 0 to 100. */
        public int percentFound() {
            return tagsInCluster == 0 ? 0 : (100 * tagsFound) / tagsInCluster;
        }
    }

    /** Any alliance: look at every cluster. */
    public static Result identify(HuskyLens.Block[] blocks) {
        return identify(blocks, null);
    }

    /**
     * Pick the cluster that has the most of its tags visible.
     * @param blocks what the HuskyLens sees (from TagCamera.getBlocks())
     * @param alliance "red" or "blue": only clusters whose name starts with this are considered.
     *                 null = consider every cluster.
     */
    public static Result identify(HuskyLens.Block[] blocks, String alliance) {
        String bestName = null;
        int bestFound = 0;
        int bestTotal = 0;

        for (Cluster cluster : CLUSTERS) {
            // Skip the other alliance's clusters, so the robot never targets them.
            if (alliance != null && !cluster.name.toLowerCase().startsWith(alliance.toLowerCase())) {
                continue;
            }
            int found = 0;
            for (int tagId : cluster.tagIds) {
                if (seen(blocks, tagId)) found++;   // count each tag once, even if it shows up twice
            }
            // Keep the cluster with the most tags visible. On a tie, the first one in the list wins.
            if (found > bestFound) {
                bestFound = found;
                bestTotal = cluster.tagIds.length;
                bestName = cluster.name;
            }
        }

        if (bestFound < MIN_TAGS_TO_MATCH) {
            return new Result(null, 0, 0);     // no cluster in view
        }
        return new Result(bestName, bestFound, bestTotal);
    }

    /** True if a block with this tag ID is in the list. */
    private static boolean seen(HuskyLens.Block[] blocks, int tagId) {
        for (HuskyLens.Block block : blocks) {
            if (block.id == tagId) return true;
        }
        return false;
    }
}
