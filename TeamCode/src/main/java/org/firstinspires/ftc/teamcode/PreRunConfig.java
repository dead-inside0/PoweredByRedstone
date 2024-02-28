public class PreRunConfig extends OpMode{
    public enum ALLIANCE {
        RED,
        BLUE;
        private static CONFIG_STAGE[] values = values();
        public CONFIG_STAGE next() {return values[(this.ordinal() + 1) % values.length]};
        public CONFIG_STAGE prev() {return values[(this.ordinal()-1+values.length) % values.length]};
    }

    public enum POSITION {
        LEFT,
        RIGHT;
        private static CONFIG_STAGE[] values = values();
        public CONFIG_STAGE next() {return values[(this.ordinal() + 1) % values.length]};
        public CONFIG_STAGE prev() {return values[(this.ordinal()-1+values.length) % values.length]};
    }

    public enum CONFIG_STAGE {
        GAMEPAD1,
        GAMEPAD2,
        ALLIANCE,
        POSITION,
        DELAY,
        CONFIRM;
        private static CONFIG_STAGE[] values = values();
        public CONFIG_STAGE next() {return values[(this.ordinal() + 1) % values.length]};
        public CONFIG_STAGE prev() {return values[(this.ordinal()-1+values.length) % values.length]};
    }

    public class Config {
        double delay;
        bool gamepad1Ready;
        bool gamepad2Ready;
        CONFIG_STAGE currConfig;
        ALLIANCE alliance;
        POSITION position;
        void next() {this.currConfig = this.currConfig.next()};
        void prev() {this.currConfig = this.currConfig.next()};
        void load() {
            try {
                InputStream inputStream = context.openFileInput("Config.txt");
                if(inputStream != null) {
                    InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                    BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                    this.ALLIANCE = ALLIANCE.valueOf(bufferedReader.readLine());
                    this.position = POSITION.vauleOf(bufferedReader.readLine());
                }
            }
        }

        public Config(ALLIANCE alliance, POSITION position, double delay) {
            this.alliance = alliance;
            this.position = position;
            this.delay = delay;
        }
    }

    Config config;

    @Override
    public void init() {
        try {
            InputStream inputStream = context.openFileInput("Config.txt");
            if(inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                ALLIANCE alliance = ALLIANCE.valueOf(bufferedReader.readLine());
                POSITION position = POSITION.valueOf(bufferedReader.readLine());
                double delay = double.valueOf(bufferedReader.readLine());
                config = new Config(alliance,position,delay);
            }
            inputStream.close();
        }
        catch (Exception e) {
            telemetry.addData("Exception", "Error reading config file: " + e.toString() + ", writing default values");
            config = new Config(ALLIANCE.RED,POSITION.LEFT,0);
        }
    }

    @Override
    public void loop() {
        boolean back1 = gamepad1.back || gamepad1.left_bumper;
        boolean a1 = gamepad1.a;
        boolean b2 = gamepad2.b;
        boolean up = gamepad1.up;
        boolean down = gamepad1.down;
    }
}