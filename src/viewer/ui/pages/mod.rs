use bevy::prelude::Resource;
use bevy_egui::egui;

use super::controls::ControlTab;
use crate::viewer::UiState;
use crate::viewer::live::Live;

pub(super) mod driving;
pub(super) mod start;
pub(super) mod tutorial;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum Page {
    #[default]
    Start,
    Tutorial,
    Driving,
}

pub(super) struct PageContext<'a> {
    pub(super) root: &'a mut egui::Ui,
    pub(super) state: &'a mut UiState,
    pub(super) live: &'a mut Live,
    pub(super) active_tab: &'a mut ControlTab,
}

#[derive(Default)]
pub(super) struct PageOutput {
    pub(super) driving_rect: Option<egui::Rect>,
    pub(super) route: Option<Route>,
}

pub(super) trait PageView {
    fn show(&mut self, context: PageContext<'_>) -> PageOutput;
}

#[derive(Clone, Copy)]
pub(super) enum Route {
    StartMenu,
    TrackSelect,
    Tutorial,
    Driving,
}

#[derive(Default)]
pub(crate) struct Pages {
    start: start::StartPage,
    tutorial: tutorial::TutorialPage,
    driving: driving::DrivingPage,
}

#[derive(Resource, Default)]
pub(crate) struct Navigator {
    page: Page,
}

impl Navigator {
    pub(crate) fn page(&self) -> Page {
        self.page
    }

    #[cfg(test)]
    pub(crate) fn navigate(&mut self, page: Page) {
        self.page = page;
    }

    pub(crate) fn show(
        &mut self,
        root: &mut egui::Ui,
        pages: &mut Pages,
        state: &mut UiState,
        live: &mut Live,
        active_tab: &mut ControlTab,
    ) -> Option<egui::Rect> {
        let context = PageContext {
            root,
            state,
            live,
            active_tab,
        };
        let output = match self.page {
            Page::Start => pages.start.show(context),
            Page::Tutorial => pages.tutorial.show(context),
            Page::Driving => pages.driving.show(context),
        };
        self.apply(output.route, pages);
        output.driving_rect
    }

    fn apply(&mut self, route: Option<Route>, pages: &mut Pages) {
        let Some(route) = route else { return };
        match route {
            Route::StartMenu => pages.start.show_menu(),
            Route::TrackSelect => pages.start.select_track(),
            Route::Tutorial => self.page = Page::Tutorial,
            Route::Driving => self.page = Page::Driving,
        }
        if matches!(route, Route::StartMenu | Route::TrackSelect) {
            self.page = Page::Start;
        }
    }

    #[cfg(test)]
    pub(super) fn show_driving(&mut self, pages: &mut Pages, context: PageContext<'_>) -> egui::Rect {
        let output = pages.driving.show(context);
        self.apply(output.route, pages);
        output.driving_rect.unwrap()
    }
}

#[cfg(test)]
impl Pages {
    pub(crate) fn select_track(&mut self) {
        self.start.select_track();
    }

    pub(crate) fn selecting_track(&self) -> bool {
        self.start.selecting_track()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navigation_has_one_unambiguous_current_page() {
        let mut navigator = Navigator::default();
        assert_eq!(navigator.page(), Page::Start);

        navigator.navigate(Page::Driving);
        assert_eq!(navigator.page(), Page::Driving);

        navigator.navigate(Page::Tutorial);
        assert_eq!(navigator.page(), Page::Tutorial);
        navigator.navigate(Page::Start);
        assert_eq!(navigator.page(), Page::Start);
    }
}
