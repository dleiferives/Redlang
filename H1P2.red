@name = H1P2;
@initial{
    scoreboard INPUT;
    collection #L1{
        score INPUT score;
        dataref ref block 992 56 8;
        pos line2 990 56 10;
        pos line1 994 56 10;
        pos abs start ~0 ~0 ~0;
        score INPUT score2;
        dataref ref2 block 981 56 8;
        pos line4 979 56 10;
        pos line3 983 56 10;
    };
};

@always[REDSTONE](
    #L1.score = #L1.ref.get(Page);
    #L1.score2 = #L1.ref2.get(Page);
){
    case(#L1.score){
        '5','0' => {
            #L1.line1.setblock(minecraft:air);
            #L1.line2.setblock(minecraft:air);
        };
        '1' => {
            #L1.line1.setblock(minecraft:air);
            #L1.line2.setblock(minecraft:redstone_block);
        };
        '2' => {
            #L1.line1.setblock(minecraft:redstone_block);
            #L1.line2.setblock(minecraft:air);
        };
        '3' => {
            #L1.line1.setblock(minecraft:redstone_block);
            #L1.line2.setblock(minecraft:redstone_block);
        };

    };
    case(#L1.score2){
        '0' => {
            #L1.line3.setblock(minecraft:air);
            #L1.line4.setblock(minecraft:air);
        };
        '1' => {
            #L1.line3.setblock(minecraft:air);
            #L1.line4.setblock(minecraft:redstone_block);
        };
        '2' => {
            #L1.line3.setblock(minecraft:redstone_block);
            #L1.line4.setblock(minecraft:air);
        };
        '3' => {
            #L1.line3.setblock(minecraft:redstone_block);
            #L1.line4.setblock(minecraft:redstone_block);
        };
    };
};

@end{
    #L1.start.setblock(minecraft:lava[level=15]);
};
