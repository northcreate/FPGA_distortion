-- Created by IP Generator (Version 2025.2 build 211867)
-- Instantiation Template
--
-- Insert the following codes into your VHDL file.
--   * Change the_instance_name to your own instance name.
--   * Change the net names in the port map.


COMPONENT pll
  PORT (
    clkin1 : IN STD_LOGIC;  -- 50.0MHz
    pll_lock : OUT STD_LOGIC;
    clkout0 : OUT STD_LOGIC;  -- 25.0MHz
    clkout1 : OUT STD_LOGIC;  -- 9.978991596638656MHz
    clkout2 : OUT STD_LOGIC  -- 24.739583333333332MHz
  );
END COMPONENT;


the_instance_name : pll
  PORT MAP (
    clkin1 => clkin1,
    pll_lock => pll_lock,
    clkout0 => clkout0,
    clkout1 => clkout1,
    clkout2 => clkout2
  );
